import time

import flask
import logging

import shared_util

import motors.consts
import server.consts


def create_response(value):
    response = flask.jsonify(value)
    response.headers.add("Access-Control-Allow-Origin", "*")
    return response


def thread(server_dq, server_motor_dq):
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    app = flask.Flask(__name__)

    fps_controller = shared_util.FPSController()

    server_ds = shared_util.DoubleState(server.consts.STATE_FROM_MASTER, server.consts.STATE_FROM_SELF)
    server_motor_ds = shared_util.DoubleState(motors.consts.STATE_FROM_SERVER, motors.consts.STATE_FROM_SELF)

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
        server_ds.s2["invert"] = value == "true"
        server_ds.put_s2(server_dq)
        return create_response(value)

    @app.route("/config/<type>/<key>/<value>", methods=["GET"])
    def config(type, key, value):
        if type == "motor":
            if key == "velocity_limit":
                server_motor_ds.s1["velocity_limit"]["value"] = int(value)
                server_motor_ds.s1["velocity_limit"]["count"] += 1
                server_motor_ds.put_s1(server_motor_dq)
            else:
                server_motor_ds.s1[key] = int(value)
                server_motor_ds.put_s1(server_motor_dq)
        else:
            try:
                server_ds.s2[key] = int(value)
            except ValueError:
                server_ds.s2[key] = float(value)
            server_ds.put_s2(server_dq)

        return create_response(value)
    
    @app.route("/write_every_frame/<value>", methods=["GET"])
    def write_every_frame(value):
        server_motor_ds.s1["write_every_frame"] = value == "true"
        server_motor_ds.put_s1(server_motor_dq)
        return create_response(value)
    
    @app.route("/power/<left>/<right>/", methods=["GET"])
    def power(left, right):
        # Has percent power built into values
        server_motor_ds.s1["left"]  = float(left)
        server_motor_ds.s1["right"] = float(right)
        server_motor_ds.s1["count"] += 1
        server_motor_ds.put_s1(server_motor_dq)

        return create_response({
            "left": left,
            "right": right,
        })
    
    @app.route("/run/<detection>/<state>/", methods=["GET"])
    def run(detection, state):
        server_ds.s2["run"][detection] = state == "true"
        server_ds.put_s2(server_dq)

        return create_response({
            "detection": detection,
            "state": state,
        })

    @app.route("/clear/<detection>/", methods=["GET"])
    def clear(detection):
        server_ds.s2["clear"][detection] += 1
        server_ds.put_s2(server_dq)
        return create_response(detection)
    
    @app.route("/get", methods=["GET"])
    def get():
        server_ds.update_s1(server_dq)
        server_motor_ds.update_s2(server_motor_dq)
        # print("B", server_motor_ds.s2["motors"]["current"]["left"])

        server_motor_ds.s1["last_get"] = time.time()
        server_motor_ds.put_s1(server_motor_dq)

        # combine main info with motor info
        # server_ds.s1.update(server_motor_ds.s2)
        # print("C", server_ds.s1["motors"]["current"]["left"])
        server_ds.s1["fpses"][-2] = server_motor_ds.s2["motor_fps"]

        fps_controller.update()
        server_ds.s1["fpses"][-1] = fps_controller.fps()

        js_response_dict = server_ds.s1.copy()
        js_response_dict["motors"] = server_motor_ds.s2["motors"].copy()
        print("B", js_response_dict["motors"]["current"]["right"])

        return create_response(js_response_dict)


    app.run(debug=False, port=5000, host="0.0.0.0", threaded=False, processes=1)