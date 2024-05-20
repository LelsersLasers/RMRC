import time

import flask
import logging

import shared_util

import server.util

import motors.consts
import server.primary_server.consts

import pickle


def thread(primary_server_dq, primary_server_motor_dq):
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    app = flask.Flask(__name__)

    fps_controller = shared_util.FPSController()

    primary_server_ds = shared_util.DoubleState(server.primary_server.consts.STATE_FROM_MASTER, server.primary_server.consts.STATE_FROM_SELF)
    server_motor_ds = shared_util.DoubleState(motors.consts.STATE_FROM_SERVER, pickle.dumps(motors.consts.STATE_FROM_SELF))

    @app.route("/")
    def index():
        return flask.render_template("index.html")
    
    @app.route("/calibrate", methods=["GET"])
    def calibrate():
        now = time.time()
        response = flask.jsonify(now)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
    @app.route("/invert/<value>", methods=["GET"])
    def invert(value):
        primary_server_ds.s2["invert"] = value == "true"
        primary_server_ds.put_s2(primary_server_dq)
        return server.util.create_response(value)

    @app.route("/config/<type>/<key>/<value>", methods=["GET"])
    def config(type, key, value):
        if type == "motor":
            if key == "velocity_limit":
                server_motor_ds.s1["velocity_limit"]["value"] = int(value)
                server_motor_ds.s1["velocity_limit"]["count"] += 1
                server_motor_ds.put_s1(primary_server_motor_dq)
            else:
                server_motor_ds.s1[key] = int(value)
                server_motor_ds.put_s1(primary_server_motor_dq)
        else:
            try:
                primary_server_ds.s2[key] = int(value)
            except ValueError:
                primary_server_ds.s2[key] = float(value)
            primary_server_ds.put_s2(primary_server_dq)

        return server.util.create_response(value)
    
    @app.route("/write_every_frame/<value>", methods=["GET"])
    def write_every_frame(value):
        server_motor_ds.s1["write_every_frame"] = value == "true"
        server_motor_ds.put_s1(primary_server_motor_dq)
        return server.util.create_response(value)
    
    @app.route("/power/<left>/<right>/", methods=["GET"])
    def power(left, right):
        # Has percent power built into values
        server_motor_ds.s1["left"]  = float(left)
        server_motor_ds.s1["right"] = float(right)
        server_motor_ds.s1["count"] += 1
        server_motor_ds.put_s1(primary_server_motor_dq)

        return server.util.create_response({
            "left": left,
            "right": right,
        })
    
    @app.route("/arm_active/<value>", methods=["GET"])
    def arm_active(value):
        server_motor_ds.s1["arm_active"] = value == "true"
        server_motor_ds.put_s1(primary_server_motor_dq)
        return server.util.create_response(value)

    @app.route("/run/<detection>/<state>/", methods=["GET"])
    def run(detection, state):
        primary_server_ds.s2["run"][detection] = state == "true"
        primary_server_ds.put_s2(primary_server_dq)
        return server.util.create_response({
            "detection": detection,
            "state": state,
        })

    @app.route("/clear/<detection>/", methods=["GET"])
    def clear(detection):
        primary_server_ds.s2["clear"][detection] += 1
        primary_server_ds.put_s2(primary_server_dq)
        return server.util.create_response(detection)
    
    @app.route("/get", methods=["GET"])
    def get():
        start = time.time()

        primary_server_ds.update_s1(primary_server_dq)
        server_motor_ds.update_s2(primary_server_motor_dq)
        
        # Note: a pickled dict is directly put into q2 instead of the original dict
        unpickled_server_motor_ds_s2 = pickle.loads(server_motor_ds.s2)

        server_motor_ds.s1["last_get"] = time.time()
        server_motor_ds.put_s1(primary_server_motor_dq)

        # combine main info with motor info
        primary_server_ds.s1.update(unpickled_server_motor_ds_s2)
        primary_server_ds.s1["fpses"][-3] = unpickled_server_motor_ds_s2["motor_fps"]
        primary_server_ds.s1["fpses"][-2] = unpickled_server_motor_ds_s2["arm_reader_fps"]

        fps_controller.update()
        primary_server_ds.s1["fpses"][-1] = fps_controller.fps()

        end = time.time()

        print("GET:", (end - start) * 1000, "ms")

        return server.util.create_response(primary_server_ds.s1)


    app.run(debug=False, port=server.primary_server.consts.PORT, host="0.0.0.0", threaded=False, processes=1)