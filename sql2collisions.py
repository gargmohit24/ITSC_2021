#!/usr/bin/env python

import argparse
import json
import os
import sqlite3
import sys
import xml

from collections import defaultdict, namedtuple
from copy import deepcopy
from xml.sax.handler import ContentHandler
from statistics import stdev
from shapely.geometry import Point, LineString
from shapely.strtree import STRtree
from shapely import speedups

DEBUG = False
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

# EdgeLookup is a class for looking up the lane on which a given vehicle is travelling
class EdgeLookup:
    def __init__(self, netfile: str, xformer: CoordTransformer):
        xp = xml.sax.make_parser()
        xp.setFeature(xml.sax.handler.feature_namespaces, 0)
        r = NetReader()
        xp.setContentHandler(r)
        xp.parse(netfile)
        self._lanes = deepcopy(r._data)
        self._lane_lookup = dict([(id(l['points']), l) for l in self._lanes])
        self._edge_rtree = STRtree([l['points'] for l in self._lanes])
        self._xformer = xformer

    def find(self, pos: Point, threshold: float = 0.1):
        traciPos = self._xformer.omnet2traci(pos)
        nearest = self._edge_rtree.nearest(traciPos)
        if nearest.distance(traciPos) > threshold:
            raise ValueError(f'{traciPos.wkt} too far away from {nearest.wkt}')
        return self._lane_lookup.get(id(nearest))


# ProximityHelper is a helper class for locating things within a certain radius
class ProximityHelper:
    # data should consist of [(thing_id, Point(thing_posx, thing_posy)], ...]
    def __init__(self, data):
        d = dict([(row[0], row[1]) for row in data])
        self._lookup = dict([(id(point), thing_id) for thing_id, point in d.items()])
        self._rtree = STRtree(d.values())

    # Find returns all things and their locations within threshold of pos.
    def find(self, pos: Point, threshold: float = 10.0):
        out = []
        query_geom = pos.buffer(threshold)
        for result in self._rtree.query(query_geom):
            thing_id = self._lookup[id(result)]
            out.append((thing_id, result))
        return out


def is_following(up: Point, uc: Point, vp: Point, vc: Point) -> bool:
    """For vehicles u and v, given previous and current positions up, uc, vp, vc
    respectively, return True if u is behind (following) v, and False otherwise. 

    # u behind v, equal speeds
    >>> is_following(Point(0, 0), Point(1, 0), Point(3, 0), Point(4, 0))
    True
    >>> is_following(Point(0, 0), Point(1, 0), Point(1, 0), Point(2, 0))
    True

    # u ahead of v, equal speeds
    >>> is_following(Point(3, 0), Point(4, 0), Point(0, 0), Point(1, 0))
    False
    >>> is_following(Point(1, 0), Point(2, 0), Point(0, 0), Point(1, 0))
    False

    # u travelling in an opposite direction to v
    >>> is_following(Point(2, 0), Point(1, 0), Point(3, 0), Point(4, 0))
    False

    # u behind v, v faster than u
    >>> is_following(Point(0, 0), Point(1, 0), Point(1, 0), Point(3, 0))
    False

    # u ahead of v, v faster than u
    >>> is_following(Point(3, 0), Point(4, 0), Point(0, 0), Point(3, 0))
    False

    # u behind v, u faster than v
    >>> is_following(Point(0, 0), Point(3, 0), Point(2, 0), Point(4, 0))
    True

    # u ahead of v, u faster than v
    >>> is_following(Point(2, 0), Point(4, 0), Point(0, 0), Point(1, 0))
    False
    """

    return (up.distance(vc) > up.distance(vp)) and (uc.distance(vc) <= up.distance(vp))

# Vehicle represents a vehicle at a given point in time
Vehicle = namedtuple('Vehicle', ['prev_pos', 'curr_pos', 'speed', 'controller'])

# A Snapshot consists of vehicle data within two discrete instants i, j, i < j.
class Snapshot:
    def __init__(self, cursor, run_id, start, end):
        self._data = {}
        start_data = {}
        end_data = {}
        sql = '''
            select node_id, controller, mobility_posx, mobility_posy, appl_speed
            from results
            where run_id = ?
            and seconds = ?
            and mobility_posx is not null
            and mobility_posy is not null
            and appl_speed is not null;
        '''

        start_rows = cursor.execute(sql, (run_id, start))
        for row in start_rows:
            node_id, controller, xpos, ypos, _ = row
            start_data[node_id] = (Point(xpos, ypos), controller)

        end_rows = cursor.execute(sql, (run_id, end))
        for row in end_rows:
            node_id, controller, xpos, ypos, speed = row
            end_data[node_id] = (Point(xpos, ypos), speed, controller)

        # we only care about vehicles in both start and end instants
        common_ids = set(start_data.keys()).intersection(end_data.keys())
        for common_id in common_ids:
            self._data[common_id] = Vehicle(start_data[common_id][0], end_data[common_id][0], end_data[common_id][1], end_data[common_id][2])
        dbg(f"snapshot init: run_id:{run_id} start:{start:.2f} end:{end:.2f} start_rows:{len(start_data.keys())} end_rows:{len(end_data.keys())} common_ids:{len(common_ids)}")

    def vehicle_ids(self):
        return self._data.keys()

    def get(self, vehicle_id):
        return self._data.get(vehicle_id)

    def items(self):
        return self._data.items()

    def start(self):
        return [(k, v[0]) for k, v in self._data.items()]

    def end(self):
        return [(k, v[1]) for k, v in self._data.items()]

    def __len__(self):
        return len(self._data.items())

__HELP__ = '''Look for collisions in a traffic dataset.\n

Sample invocation: sql2collisions.py --run_id 9 --start_time 25201 --end_time 25250 --xform_x1 679.56 --xform_y1 966.00 --xform_x2 4441.09 --xform_y2 9242.02 --xform_margin 25 hightraffic.db net.xml
'''


def main():
    parser = argparse.ArgumentParser(description=__HELP__)
    parser.add_argument("dbpath", help="path to sqlite database file")
    parser.add_argument("netfile", help="path to sumo network file")
    parser.add_argument("--xform_x1", type=float, default=679.56, help="top-left x value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_y1", type=float, default=966.00, help="top-left y value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_x2", type=float, default=4441.09, help="bottom-right x value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_y2", type=float, default=9242.02, help="bottom-right y value for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--xform_margin", type=float, default=25.0, help="margin for coordinate transformation between OMNeT++ and SUMO")
    parser.add_argument("--run_id", type=int, default=0, help="run number")
    parser.add_argument("--debug", default=False, action='store_true', help="debug output")
    parser.add_argument("--start_time", type=float, help="start time (inclusive)")
    parser.add_argument("--end_time", type=float, help="end time (exclusive)")
    parser.add_argument("--ttc", type=float, help="time to collision", default=1.0)
    parser.add_argument("--all_snapshots", default=False, action='store_true', help='process all snapshots, not only whole seconds')
    args = parser.parse_args()

    global DEBUG
    DEBUG = args.debug

    dbg("shapely speedups available: {speedups.enabled}")

    xformer = CoordTransformer(Point(args.xform_x1, args.xform_y1), Point(args.xform_x2, args.xform_y2), args.xform_margin)
    edge_lookup = EdgeLookup(args.netfile, xformer)

    db_uri = f"file://{os.path.abspath(os.path.expanduser(args.dbpath))}?mode=rw"
    conn = sqlite3.connect(db_uri, uri=True)
    c = conn.cursor()

    sql_create_table_colisions = '''
    create table if not exists collisions (
        leader_node_id integer,
        follower_node_id integer,
        seconds real,
        lane_id string,
        primary key (leader_node_id, follower_node_id, seconds)
    );
    '''
    c.execute(sql_create_table_colisions)
    sql_insert_collision = 'insert or ignore into collisions values (?, ?, ?, ?);'

    start_time = args.start_time
    if start_time is None:
        start_time = c.execute('select min(seconds) from results where run_id = ?', (args.run_id,)).fetchone()[0]

    end_time = args.end_time
    if end_time is None:
        end_time = c.execute('select max(seconds) from results where run_id = ?', (args.run_id,)).fetchone()[0]

    sql_instants = '''
        select distinct seconds from results
        where seconds >= ?
        and seconds <= ?
        order by seconds asc;
    '''
    instants = [float(row[0]) for row in c.execute(sql_instants, (args.start_time, args.end_time))]
    # HACK: for some reason, snapshots that are a fraction of a second after a whole second have incomplete data. Ignore these.
    if not args.all_snapshots:
        instants = [i for i in instants if i.is_integer()]
    dbg(f"found {len(instants)} instants between {args.start_time:.2f} and {args.end_time:.2f}")
    for (prev_instant, curr_instant) in zip(instants, instants[1:]):
        snapshot = Snapshot(c, args.run_id, prev_instant, curr_instant)
        # Create a temporary rtree containing only node_ids in s_curr, mapping their node_ids to their xy positions
        ph = ProximityHelper(snapshot.end())
        print(f"processing snapshot run_id:{args.run_id} start:{prev_instant:.2f} end:{curr_instant:.2f} vehicles:{len(snapshot)}")
        # For each node_id N in instant:
        for vehicle_id, vehicle in snapshot.items():
            # Get current edge En for N from edge_rtree
            vehicle_lane_id = edge_lookup.find(vehicle.curr_pos)['lane_id']
            # Set danger radius equal to v_current * ttc_boundary
            ttc_boundry = vehicle.speed * args.ttc
            # Query temporary rtree for node_ids in danger radius
            nearby_vehicles = ph.find(vehicle.curr_pos, ttc_boundry)
            dbg(f"vehicle {vehicle_id}@{vehicle.curr_pos.wkt} has {len(nearby_vehicles)} foes within {ttc_boundry:.2f}")
            # For each node_id M in danger radius:
            for nearby_vehicle_id, _ in nearby_vehicles:
                if nearby_vehicle_id == vehicle_id:
                    continue
                nearby_vehicle = snapshot.get(nearby_vehicle_id)
                # Get current edge Em for M from edge_rtree
                nearby_vehicle_lane_id = edge_lookup.find(nearby_vehicle.curr_pos)['lane_id']
                # If En and Em differ, ignore
                if nearby_vehicle_lane_id != vehicle_lane_id:
                    dbg(f"ignoring nearby vehicle {nearby_vehicle_id}@{nearby_vehicle.curr_pos.wkt} for vehicle {vehicle_id}@{vehicle.curr_pos.wkt} on {vehicle_lane_id} ")
                    continue
                # If N is not following M, ignore -- we will catch M later
                if not is_following(vehicle.prev_pos, vehicle.curr_pos, nearby_vehicle.prev_pos, nearby_vehicle.curr_pos):
                    dbg(f"vehicle {vehicle_id}@{vehicle.curr_pos.wkt} is not following vehicle {nearby_vehicle_id}@{nearby_vehicle.curr_pos.wkt}")
                    continue
                # At this point, N and M are known to be on the same lane,
                # N is known to be following M, and both N and M are within
                # an unsafe stopping distance.
                # One of them is going to crash into the other.
                # Record a collision between N (follower) and M (leader)
                # TODO: write to database
                dbg(f"ttc < {args.ttc:.2f} between {vehicle_id}@{vehicle.curr_pos.wkt} travelling at {vehicle.speed:.2f} and {nearby_vehicle_id}@{nearby_vehicle.curr_pos.wkt}")
                c.execute(sql_insert_collision, (nearby_vehicle_id, vehicle_id, curr_instant, vehicle_lane_id))
                conn.commit()


if __name__ == "__main__":
    main()

# vim: set ts=4 sw=4 expandtab:
