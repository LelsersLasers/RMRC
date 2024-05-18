import flask
import logging

import shared_util

import server.arm_server.consts
import server.util

def thread(arm_server_motor_sq):
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    app = flask.Flask(__name__)

    arm_server_motor_ss = shared_util.SingleState(server.arm_server.consts.STATE_FROM_SELF)

    @app.route("/arm/<j1>/<j2>/<j3>/<fps>/<time>", methods=["GET"])
    def arm(j1, j2, j3, fps, time):
        arm_server_motor_ss.s["arm_target_positions"]["j1"] = int(j1)
        arm_server_motor_ss.s["arm_target_positions"]["j2"] = int(j2)
        arm_server_motor_ss.s["arm_target_positions"]["j3"] = int(j3)
        arm_server_motor_ss.s["arm_reader_fps"] = float(fps)
        arm_server_motor_ss.s["time"] = float(time)
        arm_server_motor_ss.put_s(arm_server_motor_sq)

        return server.util.create_response({
            "j1": j1,
            "j2": j2,
            "j3": j3,
            "fps": fps,
        })
    
    app.run(debug=False, port=server.arm_server.consts.PORT, host="0.0.0.0", threaded=False, processes=1)