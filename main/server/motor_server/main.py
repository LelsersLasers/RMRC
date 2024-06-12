import flask
import logging

import threading

import shared_util

import server.motor_server.consts
import server.util

def process(motor_server_motor_dq):
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    app = flask.Flask(__name__)

    motor_server_motor_ds = shared_util.DoubleState(server.motor_server.consts.STATE_FROM_SELF, server.motor_server.consts.STATE_FROM_MOTORS)

    motor_server_motor_s1_lock = threading.Lock()

    # ------------------------------------------------------------------------ #
    @app.route("/power_percent/<decimal>", methods=["GET"])
    def power_percent(decimal):
        with motor_server_motor_s1_lock:
            motor_server_motor_ds.s1["power_percent"] = float(decimal)
            motor_server_motor_ds.put_s1(motor_server_motor_dq)
        return server.util.create_response(decimal)

    @app.route("/power/<left>/<right>/", methods=["GET"])
    def power(left, right):
        # Does not percent power built into values
        with motor_server_motor_s1_lock:
            motor_server_motor_ds.s1["left"]  = float(left)
            motor_server_motor_ds.s1["right"] = float(right)
            motor_server_motor_ds.s1["count"] += 1
            motor_server_motor_ds.put_s1(motor_server_motor_dq)

        motor_server_motor_ds.update_s2(motor_server_motor_dq)
        invert = motor_server_motor_ds.s2["invert"]

        return server.util.create_response({
            "left": left,
            "right": right,
            "invert": invert,
        })
    # ------------------------------------------------------------------------ #
    
    # ------------------------------------------------------------------------ #
    @app.route("/cycles/<j1>/<j2>/<j3>", methods=["GET"])
    def cycles(j1, j2, j3):
        with motor_server_motor_s1_lock:
            motor_server_motor_ds.s1["cycles"]["j1"] = int(j1)
            motor_server_motor_ds.s1["cycles"]["j2"] = int(j2)
            motor_server_motor_ds.s1["cycles"]["j3"] = int(j3)
            motor_server_motor_ds.put_s1(motor_server_motor_dq)

            return server.util.create_response({
                "j1": motor_server_motor_ds.s1["cycles"]["j1"],
                "j2": motor_server_motor_ds.s1["cycles"]["j2"],
                "j3": motor_server_motor_ds.s1["cycles"]["j3"]
            })

    @app.route("/joints/<j1>/<j2>/<j3>/<fps>/<time>", methods=["GET"])
    def joints(j1, j2, j3, fps, time):
        with motor_server_motor_s1_lock:
            motor_server_motor_ds.s1["arm_target_positions"]["j1"] = int(j1)
            motor_server_motor_ds.s1["arm_target_positions"]["j2"] = int(j2)
            motor_server_motor_ds.s1["arm_target_positions"]["j3"] = int(j3)
            motor_server_motor_ds.s1["arm_reader_fps"] = float(fps)
            motor_server_motor_ds.s1["arm_time"] = float(time)
            motor_server_motor_ds.put_s1(motor_server_motor_dq)

        motor_server_motor_ds.update_s2(motor_server_motor_dq)
        arm_active = motor_server_motor_ds.s2["arm_active"]
        high_send_rate = motor_server_motor_ds.s2["high_send_rate"]

        return server.util.create_response({
            "arm_active": arm_active,
            "high_send_rate": high_send_rate,
        })
    # ------------------------------------------------------------------------ #
    

    app.run(debug=False, port=server.motor_server.consts.PORT, host="0.0.0.0", threaded=True, processes=1)