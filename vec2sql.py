#!/usr/bin/env python3

import argparse
import re
import sqlite3


VEC_COLS = set([s.strip() for s in '''
    mobility_posx
    mobility_posy
    mobility_acceleration
    mobility_co2emission
    appl_posx
    appl_posy
    appl_speed
    appl_acceleration
    appl_leaderDistance
    appl_relativeSpeed
    appl_controllerAcceleration
    appl_distanceTravelled
    appl_laneIdx
    prot_nodeId
    prot_busyTime
    prot_collisions
'''.split('\n') if s])

SQL_CREATE_TABLE = '''
create table if not exists results (
    node_id integer,
    run_id  integer,
    seconds real,
    frame_error_rate real,
    controller text,
    mpr real,
    mobility_posx real,
    mobility_posy real,
    mobility_acceleration real,
    mobility_co2emission real,
    appl_posx real,
    appl_posy real,
    appl_speed real,
    appl_acceleration real,
    appl_leaderDistance real,
    appl_relativeSpeed real,
    appl_controllerAcceleration real,
    appl_distanceTravelled real,
    appl_laneIdx integer,
    prot_nodeId integer,
    prot_busyTime real,
    prot_collisions integer,
    primary key (node_id, run_id, seconds)
);
'''

RUNVARS = {}
VECIDS = {}

def handle_one(conn, f):
    nrows = 0
    try:
        s = f.readline()
        if not s.startswith('version'):
            raise ValueError('expected version line')
        version = int(s.strip().split(' ')[1])
        if version != 2:
            raise ValueError('unexpected version: ' + version)

        s = f.readline()
        if not s.startswith('run'):
            raise ValueError('expected run identifier')
        runid_str = s.strip().split(' ')[1]
        runid = int(re.search(r'^[^-]+-([0-9]+)', runid_str).group(1))
        f.seek(0)
        for line in f:
            if not line.strip():
                continue
            try:
                start, rest = line.replace('\t', ' ').strip().split(' ', maxsplit=1)
            except ValueError:
                import pdb; pdb.set_trace()
            if start == 'version':
                continue
            if start == 'run':
                continue
            if start in ['attr', 'itervar', 'param']:
                attrname, attrval = rest.split(' ', maxsplit=1)
                RUNVARS[attrname] = attrval
                continue
            if start == 'vector':
                vector_id, vector_src, vector_name, etv = rest.split(' ', maxsplit=3)
                if etv != 'ETV':
                    raise ValueError('expected ETV but got ' + etv)
                node_id = int(re.search(r'^.*?\[([0-9]+)\]', vector_src).group(1))
                VECIDS[int(vector_id)] = (node_id, vector_src, vector_name)
                continue
            if start.isdigit():
                vector_id = int(start)
                _, vectime_str, vecvalue_str = rest.split(' ', maxsplit=2)
                vectime = float(vectime_str)
                vecvalue = float(vecvalue_str)
                node_id, vector_src, vector_name = VECIDS[vector_id]
                frame_error_rate = float(RUNVARS['*.**.nic.mac1609_4.frameErrorRate'])
                controller = RUNVARS['*.node[*].scenario.controller'].replace('\\"', '')
                mpr = float(RUNVARS['**.mpr'])
                # attempt to create the row with default values ignoring duplicates
                sql = 'insert or ignore into results (node_id, run_id, seconds, frame_error_rate, controller, mpr) values (?, ?, ?, ?, ?, ?);'
                conn.execute(sql, (node_id, runid, vectime, frame_error_rate, controller, mpr))

                vecsrc_last = re.search(r'^.+?\.([^\.]+)$', vector_src).group(1)
                colname = f'{vecsrc_last}_{vector_name}'
                # ignore unknown columns
                if colname not in VEC_COLS:
                    continue

                # then attempt to update the row
                sql = f'update results set {colname} = ? where node_id = ? and run_id = ? and seconds = ?;'
                conn.execute(sql, (vecvalue, node_id, runid, vectime))
                nrows += 1
                if nrows % 100000 == 0:
                    print(f'{nrows} rows processed')

        # finally delete rows missing important data, using one column as a proxy
        sql = f'delete from results where appl_distanceTravelled is null;'
        conn.execute(sql)
    except Exception as e:
        print(e)
    finally:
        conn.commit()
    

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input', nargs="+")
    parser.add_argument('output')
    args = parser.parse_args()

    conn = sqlite3.connect(args.output)
    conn.execute(SQL_CREATE_TABLE)

    for fname in args.input:
        print(f'processing {fname}')
        with open('HighTraffic_"CACC"_mpr0.7_fer0.7_rep0.vec', 'r') as f:
            handle_one(conn, f)



if __name__ == '__main__':
    main()
