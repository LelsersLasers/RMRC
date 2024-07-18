import time

import flask
import logging
import json

import threading

import shared_util

import server.util

import motors.consts
import server.primary_server.consts

import pickle


def process(primary_server_dq, primary_server_motor_dq):
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    app = flask.Flask(__name__)

    fps_controller = shared_util.FPSController()

    primary_server_ds = shared_util.DoubleState(server.primary_server.consts.STATE_FROM_MASTER, server.primary_server.consts.STATE_FROM_SELF)
    primary_server_motor_ds = shared_util.DoubleState(motors.consts.STATE_FROM_SERVER, pickle.dumps(motors.consts.STATE_FROM_SELF))

    backend_fps_dict = {
        "backend_fps": 30,
        "times": [],
    }

    primary_server_s2_lock = threading.Lock()
    primary_server_motor_s1_lock = threading.Lock()


    @app.route("/")
    def index():
        return flask.render_template("index.html")
    
    @app.route("/time_offset/<offset>", methods=["GET"])
    def time_offset(offset):
        with primary_server_motor_s1_lock:
            primary_server_motor_ds.s1["time_offset"] = float(offset) / 1000
            primary_server_motor_ds.put_s1(primary_server_motor_dq)
        return server.util.create_response(offset)

    @app.route("/invert/<value>", methods=["GET"])
    def invert(value):
        with primary_server_motor_s1_lock:
            primary_server_motor_ds.s1["invert"] = value == "true"
            primary_server_motor_ds.put_s1(primary_server_motor_dq)

        with primary_server_s2_lock:
            primary_server_ds.s2["invert"] = value == "true"
            primary_server_ds.put_s2(primary_server_dq)
        
        return server.util.create_response(value)
    
    @app.route("/torque/<value>", methods=["GET"])
    def torque(value):
        with primary_server_motor_s1_lock:
            primary_server_motor_ds.s1["torque"] = value == "true"
            primary_server_motor_ds.put_s1(primary_server_motor_dq)
        
        return server.util.create_response(value)
    
    @app.route("/camera/<base_key>/<alt_Key>", methods=["GET"])
    def camera(base_key, alt_Key):
        with primary_server_s2_lock:
            primary_server_ds.s2["active_keys"] = [base_key, alt_Key]
            primary_server_ds.put_s2(primary_server_dq)
        return server.util.create_response([base_key, alt_Key])

    @app.route("/config/<type>/<key>/<value>", methods=["GET"])
    def config(type, key, value):
        if type == "motor":
            with primary_server_motor_s1_lock:
                if key == "velocity_limit":
                    primary_server_motor_ds.s1["velocity_limit"]["value"] = int(value)
                    primary_server_motor_ds.s1["velocity_limit"]["count"] += 1
                    primary_server_motor_ds.put_s1(primary_server_motor_dq)
                else:
                    primary_server_motor_ds.s1[key] = int(value)
                    primary_server_motor_ds.put_s1(primary_server_motor_dq)
        elif type == "backend" and key == "backend_fps":
            backend_fps_dict["backend_fps"] = int(value)
        else:
            with primary_server_s2_lock:
                try:
                    primary_server_ds.s2[key] = int(value)
                except ValueError:
                    primary_server_ds.s2[key] = float(value)
                primary_server_ds.put_s2(primary_server_dq)

        return server.util.create_response(value)
    
    @app.route("/write_every_frame/<value>", methods=["GET"])
    def write_every_frame(value):
        with primary_server_motor_s1_lock:
            primary_server_motor_ds.s1["write_every_frame"] = value == "true"
            primary_server_motor_ds.put_s1(primary_server_motor_dq)
        return server.util.create_response(value)
    
    @app.route("/arm_active/<value>", methods=["GET"])
    def arm_active(value):
        primary_server_motor_ds.s1["arm_active"] = value == "true"
        primary_server_motor_ds.put_s1(primary_server_motor_dq)
        return server.util.create_response(value)

    @app.route("/run/<detection>/<state>/", methods=["GET"])
    def run(detection, state):
        with primary_server_s2_lock:
            primary_server_ds.s2["run"][detection] = state == "true"
            primary_server_ds.put_s2(primary_server_dq)
        return server.util.create_response({
            "detection": detection,
            "state": state,
        })

    @app.route("/clear/<detection>/", methods=["GET"])
    def clear(detection):
        with primary_server_s2_lock:
            primary_server_ds.s2["clear"][detection] += 1
            primary_server_ds.put_s2(primary_server_dq)
        return server.util.create_response(detection)
    
    @app.route("/get", methods=["GET"])
    def get():
        def generate():
            while True:
                start = time.time()
                primary_server_ds.update_s1(primary_server_dq)
                primary_server_motor_ds.update_s2(primary_server_motor_dq)
                
                # Note: a pickled dict is directly put into q2 instead of the original dict
                unpickled_server_motor_ds_s2 = pickle.loads(primary_server_motor_ds.s2)

                with primary_server_motor_s1_lock:
                    primary_server_motor_ds.s1["last_get"] = time.time()
                    primary_server_motor_ds.put_s1(primary_server_motor_dq)

                # combine main info with motor info
                primary_server_ds.s1.update(unpickled_server_motor_ds_s2)
                primary_server_ds.s1["fpses"][-3] = unpickled_server_motor_ds_s2["motor_fps"]
                primary_server_ds.s1["fpses"][-2] = unpickled_server_motor_ds_s2["arm_reader_fps"]

                fps_controller.update()
                primary_server_ds.s1["fpses"][-1] = fps_controller.fps()

                primary_server_ds.s1["ping_time"] = time.time()


                frames_dict = primary_server_ds.s1["frames"].copy()
                response_dict = primary_server_ds.s1.copy()
                response_dict["frames"] = frames_dict

                for key in primary_server_ds.s1["frames"]:
                    primary_server_ds.s1["frames"][key] = ""

                yield json.dumps(response_dict) + "\n"

                pass_time = time.time() - start
                backend_fps_dict["times"].append(pass_time)
                backend_fps_dict["times"] = backend_fps_dict["times"][-server.primary_server.consts.TIMES_TO_KEEP:]
                avg_time = sum(backend_fps_dict["times"]) / len(backend_fps_dict["times"])

                target_time = 1 / backend_fps_dict["backend_fps"]
                sleep_target = target_time - avg_time
                if sleep_target > 0:
                    time.sleep(sleep_target)

        return generate(), {"Content-Type": "application/json", "Access-Control-Allow-Origin": "*"}


    app.run(debug=False, port=server.primary_server.consts.PORT, host="0.0.0.0", threaded=True, processes=1)