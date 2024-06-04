import flask
import logging

import shared_util

import server.arm_server.consts
import server.util

def process(arm_server_motor_dq):
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    app = flask.Flask(__name__)

    arm_server_motor_ds = shared_util.DoubleState(server.arm_server.consts.STATE_FROM_SELF, server.arm_server.consts.STATE_FROM_MOTORS)

    @app.route("/cycles/<j1>/<j2>/<j3>", methods=["GET"])
    def cycles(j1, j2, j3):
        arm_server_motor_ds.s1["cycles"]["j1"] = int(j1)
        arm_server_motor_ds.s1["cycles"]["j2"] = int(j2)
        arm_server_motor_ds.s1["cycles"]["j3"] = int(j3)
        arm_server_motor_ds.put_s1(arm_server_motor_dq)

        return server.util.create_response({
            "j1": arm_server_motor_ds.s1["cycles"]["j1"],
            "j2": arm_server_motor_ds.s1["cycles"]["j2"],
            "j3": arm_server_motor_ds.s1["cycles"]["j3"]
        })

    @app.route("/joints/<j1>/<j2>/<j3>/<fps>/<time>", methods=["GET"])
    def joints(j1, j2, j3, fps, time):
        arm_server_motor_ds.s1["arm_target_positions"]["j1"] = int(j1)
        arm_server_motor_ds.s1["arm_target_positions"]["j2"] = int(j2)
        arm_server_motor_ds.s1["arm_target_positions"]["j3"] = int(j3)
        arm_server_motor_ds.s1["arm_reader_fps"] = float(fps)
        arm_server_motor_ds.s1["time"] = float(time)
        arm_server_motor_ds.put_s1(arm_server_motor_dq)

        arm_server_motor_ds.update_s2(arm_server_motor_dq)
        arm_active = arm_server_motor_ds.s2["arm_active"]

        return server.util.create_response({ "arm_active": arm_active })
    
    app.run(debug=False, port=server.arm_server.consts.PORT, host="0.0.0.0", threaded=False, processes=1)