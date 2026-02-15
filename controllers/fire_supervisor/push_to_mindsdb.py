import json, os, sys
from mindsdb_sdk import connect

LOG_PATH = os.path.join(os.path.dirname(__file__), 'hover_log.jsonl')
MDB_URL = os.environ.get('MINDSDB_URL', 'https://cloud.mindsdb.com')
MDB_USER = os.environ.get('MINDSDB_USER')
MDB_PWD = os.environ.get('MINDSDB_PWD')
PROJECT = 'firebot'
TABLE = 'hover_events'

if not MDB_USER or not MDB_PWD:
    print("MINDSDB_USER and MINDSDB_PWD must be set", file=sys.stderr)
    sys.exit(1)

if not os.path.exists(LOG_PATH):
    print("No hover_log.jsonl found", file=sys.stderr)
    sys.exit(0)

mdb = connect(url=MDB_URL, username=MDB_USER, password=MDB_PWD)
proj = None
for p in mdb.list_projects():
    if p.name == PROJECT:
        proj = p
        break
if proj is None:
    proj = mdb.create_project(PROJECT)

# Create table if missing
try:
    proj.query(f"CREATE TABLE {TABLE} (ts DOUBLE, drone_gps JSON, fire_gps JSON, note TEXT);")
except Exception:
    pass

rows = [json.loads(line) for line in open(LOG_PATH)]
if not rows:
    print("No records to insert")
    sys.exit(0)

for r in rows:
    proj.query(
        f"INSERT INTO {TABLE} (ts, drone_gps, fire_gps, note) VALUES (?, ?, ?, ?);",
        params=[r.get('ts'), json.dumps(r.get('drone_gps')), json.dumps(r.get('fire_gps')), r.get('note','')]
    )

print(f"Inserted {len(rows)} records into {PROJECT}.{TABLE}")
