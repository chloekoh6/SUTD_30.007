import shelve

DB_FILE = 'actuator.db'  # This will create files like actuator.db.db, etc.

with shelve.open(DB_FILE) as db:
    db['position'] = 0         # or any default actuator position
    print("Initialized actuator.db with default values.")
