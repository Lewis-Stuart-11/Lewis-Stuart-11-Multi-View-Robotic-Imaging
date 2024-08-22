import sqlite3 as sq
import os
import datetime


class ImgCaptureDatabaseHandler():
    def __init__(self, database_path, database_name="experiments.db"):
        self.database_file = os.path.join(database_path, database_name)

        create_database = not os.path.isfile(self.database_file)

        self.conn = sq.connect(self.database_file, detect_types=sq.PARSE_DECLTYPES | sq.PARSE_COLNAMES)
        self.cur = self.conn.cursor()

        self.current_experiment = None

        if create_database:
            self.cur.execute("""
                            CREATE TABLE experiments
                            (experiment_id INTEGER PRIMARY KEY AUTOINCREMENT,
                            experiment_name VARCHAR(50) NOT NULL UNIQUE,
                            date_started timestamp);
                            """)

            self.cur.execute("""
                            CREATE TABLE points(
                            point_id INTEGER PRIMARY KEY AUTOINCREMENT,
                            point_num INTEGER NOT NULL,
                            was_valid BOOLEAN,
                            transform_matrix VARCHAR(200),
                            has_depth BOOLEAN,
                            has_mask BOOLEAN,
                            has_foreground BOOLEAN,
                            global_robot_id INTEGER,
                            experiment_id INTEGER NOT NULL,
                            FOREIGN KEY(experiment_id) REFERENCES experiments(experiment_id));
                            """)

    def get_experiment_with_name(self, experiment_name):
        experiments = self.cur.execute("SELECT * FROM experiments WHERE experiment_name='" + experiment_name + "';")

        return experiments.fetchone()

    def get_all_experiments(self):
        return self.cur.execute("SELECT * FROM experiments;").fetchall()

    def set_current_experiment(self, experiment_name):
        self.current_experiment = self.get_experiment_with_name(experiment_name)

    def create_new_experiment(self, experiment_name):

        experiment = (experiment_name, datetime.datetime.now())

        self.cur.executemany("""INSERT INTO experiments (experiment_name, date_started) VALUES (?,?)""", [experiment])

        self.conn.commit()

        self.set_current_experiment(experiment_name)

        return True

    def remove_experiment_with_name(self, experiment_name):
        experiment = self.get_experiment_with_name(experiment_name)

        if experiment is None:
            return False

        self.cur.execute("DELETE FROM experiments WHERE experiment_name='" + experiment_name + "';")
        self.cur.execute("DELETE FROM points WHERE experiment_id='" + str(experiment[0]) + "';")

        self.conn.commit()

        return True

    def get_point_with_num(self, point_num):
        point = self.cur.execute("SELECT * FROM points WHERE experiment_id='" + str(self.current_experiment[0]) +
                                 "' AND point_num='" + str(point_num) + "';")

        return point.fetchone()

    def get_points_for_experiment(self):
        experiment = self.cur.execute("SELECT * FROM points WHERE experiment_id='" + str(self.current_experiment[0]) + "';")

        return experiment.fetchall()

    def add_point_to_experiment(self, point_num: int, is_valid: bool, frame_data: dict):

        point = self.get_point_with_num(point_num)

        transform_matrix = frame_data["transform_matrix"] if "transform_matrix" in frame_data else ""
        has_depth = "1" if "depth_file_path" in frame_data else "0"
        has_mask = "1" if "mask_file_path" in frame_data else "0"
        has_foreground = "1" if "segmented_file_path" in frame_data else "0"
        global_robot_id = frame_data["global_robot_id"] if "global_robot_id" in frame_data else "0"

        if point is not None:

            if point[2] != (1 if is_valid else 0):
                self.update_point_in_experiment(point_num, is_valid, frame_data)

            return

        self.cur.execute(f"""INSERT INTO points(point_num, was_valid, transform_matrix, 
                             has_depth, has_mask, has_foreground, global_robot_id, experiment_id) 
                             VALUES('{str(point_num)}', '{"1" if is_valid else "0"}', 
                             '{transform_matrix}', '{has_depth}', '{has_mask}', 
                             '{has_foreground}', '{global_robot_id}', '{str(self.current_experiment[0])}');""")

        self.conn.commit()

    def update_point_in_experiment(self, point_num:int, is_valid: bool, frame_data: dict):

        transform_matrix = frame_data["transform_matrix"] if "transform_matrix" in frame_data else ""
        has_depth = "1" if "depth_file_path" in frame_data else "0"
        has_mask = "1" if "mask_file_path" in frame_data else "0"
        has_foreground = "1" if "segmented_file_path" in frame_data else "0"
        global_robot_id = frame_data["global_robot_id"]
        

        self.cur.execute(f"""UPDATE points SET was_valid = '{"1" if is_valid else "0"}', transform_matrix = '{transform_matrix}',
                             has_depth = '{has_depth}', has_mask = '{has_mask}', has_foreground = '{has_foreground}', global_robot_id = '{global_robot_id}'
                             WHERE point_num = '{str(point_num)}' AND experiment_id = '{str(self.current_experiment[0])}';""")

        self.conn.commit()

    def get_experiment_statistics(self):

        statistics = {"experiments": {}, "points": {}}

        experiments = self.get_all_experiments()

        for experiment in experiments:
            self.set_current_experiment(experiment[1])

            points = self.get_points_for_experiment()

            statistics["experiments"][experiment[1]] = {"experiment_data": experiment,
                                                        "points": points,
                                                        "num_attempted": len([i for i in points if i[3] == 1]),
                                                        "num_successful": len([i for i in points if i[2] == 1])}

            for point in points:
                if point[3] == 1:

                    if str(point[1]) not in statistics["points"].keys():
                        statistics["points"][str(point[1])] = {"total": 0, "successful": 0, "unsuccessful": 0}

                    statistics["points"][str(point[1])]["total"] += 1

                    if point[2] == 1:
                        statistics["points"][str(point[1])]["successful"] += 1
                    else:
                        statistics["points"][str(point[1])]["unsuccessful"] += 1

        return statistics

    def close_database(self):
        self.conn.close()