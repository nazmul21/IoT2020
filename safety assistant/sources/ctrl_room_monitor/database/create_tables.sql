PRAGMA foreign_keys=OFF;
BEGIN TRANSACTION;
CREATE TABLE access_log (
sl      INTEGER    PRIMARY KEY    AUTOINCREMENT,
fl_name CHAR(32)                  NOT NULL,
u_role  CHAR(32)                  NOT NULL,
time    timestamp  default (strftime('%s', 'now'))
);
CREATE TABLE loads (
sl          INTEGER    PRIMARY KEY    AUTOINCREMENT,
load_type   INTEGER                   NOT NULL,
load_status CHAR(3)                   NOT NULL,
time    timestamp  default (strftime('%s', 'now'))
);
CREATE TABLE sensor_data (
sl      INTEGER    PRIMARY KEY    AUTOINCREMENT,
sen_id  INTEGER                   NOT NULL,
sen_val REAL                      NOT NULL,
time    timestamp  default (strftime('%s', 'now'))
);
CREATE TABLE users (
sl      INTEGER    PRIMARY KEY    AUTOINCREMENT,
u_name  CHAR(32)                  NOT NULL,
u_pass  CHAR(32)                  NOT NULL,
fl_name CHAR(32)                  NOT NULL,
u_role  CHAR(32)                  NOT NULL,
time    timestamp  default (strftime('%s', 'now'))
);
COMMIT;
