import time

import flask
import logging

import util

import motors.consts
import server.consts


def thread(server_dq, server_motor_dq):
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    app = flask.Flask(__name__)
    server_ds = util.DoubleState(server.consts.STATE_FROM_MASTER, server.consts.STATE_FROM_SELF)
    server_motor_ds = util.DoubleState(motors.consts.STATE_FROM_SERVER, motors.consts.STATE_FROM_SELF)

    @app.route("/")
    def index():
        return flask.render_template("index.html")
    
    @app.route("/calibrate", methods=["GET"])
    def calibrate():
        now = time.time()
        response = flask.jsonify(now)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
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

        response = flask.jsonify(value)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
    @app.route("/write_every_frame/<value>", methods=["GET"])
    def write_every_frame(value):
        server_motor_ds.s1["write_every_frame"] = value == "true"
        server_motor_ds.put_s1(server_motor_dq)

        response = flask.jsonify(value)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
    @app.route("/power/<left>/<right>/", methods=["GET"])
    def power(left, right):
        # Has percent power built into values
        server_motor_ds.s1["left"] = float(left)
        server_motor_ds.s1["right"] = float(right)
        server_motor_ds.s1["count"] += 1
        server_motor_ds.put_s1(server_motor_dq)

        response = flask.jsonify({
            "left": left,
            "right": right,
        })
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
    @app.route("/run/<detection>/<state>/", methods=["GET"])
    def run(detection, state):
        server_ds.s2["run"][detection] = state == "true"
        server_ds.put_s2(server_dq)

        response = flask.jsonify({
            "detection": detection,
            "state": state,
        })
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response

    @app.route("/clear/<detection>/", methods=["GET"])
    def clear(detection):
        server_ds.s2["clear"][detection] += 1
        server_ds.put_s2(server_dq)

        response = flask.jsonify(detection)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response
    
    @app.route("/view/<view_mode>/", methods=["GET"])
    def view(view_mode):
        server_ds.s2["view_mode"] = int(view_mode)
        server_ds.put_s2(server_dq)

        response = flask.jsonify(view_mode)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response

    @app.route("/get", methods=["GET"])
    def get():
        server_ds.update_s1(server_dq)
        server_motor_ds.update_s2(server_motor_dq)

        server_motor_ds.s1["last_get"] = time.time()
        server_motor_ds.put_s1(server_motor_dq)

        # combine main info with motor info
        server_ds.s1.update(server_motor_ds.s2)
        server_ds.s1["fpses"][-1] = server_motor_ds.s2["motor_fps"]

        response = flask.jsonify(server_ds.s1)
        response.headers.add("Access-Control-Allow-Origin", "*")
        return response

    app.run(debug=False, port=5000, host="0.0.0.0")

    # TODO: should it be threaded or not?
    # app.run(debug=False, port=5000, host="0.0.0.0", threaded=False)
    # app.run(debug=False, port=5000, host="0.0.0.0", processes=1)