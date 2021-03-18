#!/usr/bin/env python

import argparse
import os
import sqlite3
import sys
import xml

from copy import deepcopy
from xml.sax.handler import ContentHandler
from statistics import stdev
from shapely.geometry import Point, LineString
from shapely.strtree import STRtree

DEBUG = True

def dbg(msg):
    if DEBUG:
        sys.stderr.write(msg)
        sys.stderr.write('\n')

# This class translates between SUMO and OMNeT++ coordinates.
# Based on VEINS' TraCICoordinateTransformation class.
class CoordTransformer:
    def __init__(self, topleft, bottomright, margin=0):
        self._dimensions = Point(bottomright.x - topleft.x, bottomright.y - topleft.y)
        self._topleft = topleft
        self._bottomright = bottomright
        self._margin = margin

    def omnet2traci(self, p):
        return Point(p.x + self._topleft.x - self._margin, self._dimensions.y - (p.y - self._topleft.y) + self._margin)
    
    def traci2omnet(self, p):
        return Point(p.x - self._topleft.x + self._margin, self._dimensions.y - (p.y - self._topleft.y) + self._margin)


# Class for reading a SUMO network XML file
class NetReader(ContentHandler):
    def __init__(self):
        self._edge_id = None
        self._data = []
    
    def startElement(self, name, attrs):
        if name == "edge":
            self._edge_id = attrs.get("id")
        
        if name == "lane":
            points = []
            for s in attrs.get("shape", "").split(" "):
                parts = s.split(",")
                p = Point(float(parts[0]), float(parts[1]))
                points.append(p) # this is a raw TraCI point

            self._data.append({
                "edge_id": self._edge_id,
                "lane_id": attrs["id"],
                "index":   int(attrs.get("index", -1)),
                "speed":   float(attrs.get("speed", -1)),
                "length":  float(attrs.get("length", -1)),
                "points":  LineString(points),
            })
            return

# returns a mapping of key -> average of d[key] in dicts
def group_by_avg(dicts, key, value):
    tmp = {}
    ret = {}
    for d in dicts:
        k = d[key]
        if k not in tmp:
            tmp[k] = list()
        tmp[k].append(d[value])
    for k, v in tmp.items():
        ret[k] = sum(v) / len(v)
    del(tmp)
    return ret


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-dbpath", help="path to sqlite database file")
    parser.add_argument("-netfile", help="path to sumo network file")
    parser.add_argument("--outfile", help="path to output XML file, defaults to stdout")
    parser.add_argument("--xform_x1", type=float, default=679.56, help="top-left x value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_y1", type=float, default=966.00, help="top-left y value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_x2", type=float, default=4441.09, help="bottom-right x value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_y2", type=float, default=9242.02, help="bottom-right y value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_margin", type=float, default=25.0, help="margin for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--run_id", type=int, default=0, help="run number")
    parser.add_argument("--debug", default=True, action='store_true', help="debug output")
    parser.add_argument("--start_time", type=float, help="start time (inclusive)")
    parser.add_argument("--end_time", type=float, help="end time (exclusive)")
    args = parser.parse_args()

    global DEBUG
    DEBUG = args.debug

    xformer = CoordTransformer(Point(args.xform_x1, args.xform_y1), Point(args.xform_x2, args.xform_y2), args.xform_margin)

    xp = xml.sax.make_parser()
    xp.setFeature(xml.sax.handler.feature_namespaces, 0)
    r = NetReader()
    xp.setContentHandler(r)
    xp.parse(args.netfile)
    lanes = deepcopy(r._data)
    lookup = dict([(id(l['points']), l) for l in lanes])
    rtree = STRtree([l['points'] for l in lanes])

    db_uri = f"file://{os.path.abspath(os.path.expanduser(args.dbpath))}?mode=ro"
    conn = sqlite3.connect(db_uri, uri=True)
    c = conn.cursor()

    start_time = args.start_time
    if start_time is None:
        start_time = c.execute('select min(seconds) from results where run_id = ?', (args.run_id,)).fetchone()[0]

    end_time = args.end_time
    if end_time is None:
        end_time = c.execute('select max(seconds) from results where run_id = ?', (args.run_id,)).fetchone()[0]

    sql = '''select node_id, seconds, mobility_posx, mobility_posy, appl_speed from results where run_id = ? and seconds >= ? and seconds < ? and mobility_posx is not null and mobility_posy is not null and appl_speed is not null order by seconds asc;'''
    query_args = (args.run_id, start_time, end_time)

    # map of edge_id -> [list of samples]
    edgedata = dict()
    # map of edge_id -> speed
    edge2speed = group_by_avg(lanes, 'edge_id', 'speed')
    # map of edge_id -> length
    edge2len = group_by_avg(lanes, 'edge_id', 'length')
    # tmplen = {}
    # tmpspeed = {}
    # for lane in lanes:
    #     edge_id = lane['edge_id']
    #     if edge_id not in tmplen:
    #         tmplen['edge_id'] = list()
    #     if edge_id not in tmpspeed:
    #         tmpspeed['edge_id'] = list()
    #     tmplen[edge_id].append(lane['length'])
    #     tmpspeed[edge_id].append(lane['speed'])
    # for k, v in tmplen.items():
    #     edge2len[k] = sum(v) / len(v)
    # for k, v in tmpspeed.items():
    #     edge2speed[k] = sum(v) / len(v)

    for row in c.execute(sql, query_args):
        node_id, seconds, posx, posy, speed = row
        traciPoint = xformer.omnet2traci(Point(posx, posy))
        nearest = rtree.nearest(traciPoint)

        lane = lookup[id(nearest)]
        edge_id = lane['edge_id']
        distance = nearest.distance(traciPoint)
        if distance > 0.1:
            raise ValueError(f'{traciPoint.wkt} too far away from {nearest.wkt}')

        if edge_id not in edgedata:
            edgedata[edge_id] = {
                'speeds': [speed],
            }
        else:
            edgedata[edge_id]['speeds'].append(speed)

    if args.outfile is None:
        outfile = sys.stdout
    else:
        outfile = open(args.outfile, 'w')
    
    # TODO: actually generate the XML element tree properly
    outfile.write("<meandata>\n")
    outfile.write(f"<interval begin=\"{start_time}\" end=\"{end_time}\">\n")
    for edge_id, edge_dict in edgedata.items():
        n = len(edge_dict['speeds'])
        avg_speed = sum(edge_dict['speeds']) / n # average speed in m/s
        min_speed, max_speed = min(edge_dict['speeds']), max(edge_dict['speeds'])
        if len(edge_dict['speeds']) > 2:
            stdev_speed = stdev(edge_dict['speeds'])
        else:
            stdev_speed = "" # hack
        travel_rate_mpkm = 1 / avg_speed * 16.667
        edge_speed = edge2speed[edge_id]
        edge_len = edge2len[edge_id]
        expected_travel_time = edge_len / edge_speed
        actual_travel_time =  edge_len / avg_speed
        cidx = (actual_travel_time - expected_travel_time) / expected_travel_time
        outfile.write(f"<edge id=\"{edge_id}\" speed=\"{edge_speed}\" length=\"{edge_len}\" avg_speed=\"{avg_speed}\" min_speed=\"{min_speed}\" max_speed=\"{max_speed}\" stdev_speed=\"{stdev_speed}\" travelrate=\"{travel_rate_mpkm}\" congestion_index=\"{cidx}\"/>\n")
    outfile.write("</interval>\n")
    outfile.write("</meandata>\n")
    outfile.close()

if __name__ == "__main__":
    main()
