from flask import Flask, render_template, jsonify
import json
import time
import argparse
import logging


MAIN_FILE = "states/state.json"
SERVER_FILE = "states/server_state.json"

FILE_DELAY = 0.0

MAIN_STATE = {
    "frame": "",
    "w": 1,
    "h": 1,
    "hazmats_found": [],
    "qr_found": [],
}
SERVER_STATE = {}

def read_state():
    global MAIN_STATE
    try:
        with open(MAIN_FILE, "r") as f:
            MAIN_STATE = json.load(f)
    except:
        time.sleep(FILE_DELAY)
        read_state()

def write_state():
    # Write state to file
    with open(SERVER_FILE, "w") as f:
        json.dump(SERVER_STATE, f)


app=Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/keyup/<key>', methods=['GET'])
def keyup(key):
    global SERVER_STATE
    SERVER_STATE[key] = False
    write_state()

    response = jsonify(SERVER_STATE)
    response.headers.add('Access-Control-Allow-Origin', '*')

    return response

@app.route('/keydown/<key>', methods=['GET'])
def keydown(key):
    global SERVER_STATE
    SERVER_STATE[key] = True
    write_state()

    response = jsonify(SERVER_STATE)
    response.headers.add('Access-Control-Allow-Origin', '*')

    return response


@app.route('/get', methods=['GET'])
def get():
    global MAIN_STATE

    read_state()
    response = jsonify(MAIN_STATE)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response

ap = argparse.ArgumentParser()
ap.add_argument("-d", "--debug", required=False, help="show debug prints", action="store_true")
args = vars(ap.parse_args())

if __name__ == "__main__":
    read_state()
    write_state()
    
    if not args["debug"]:
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)

    app.run(debug=args["debug"], port=5000, host='0.0.0.0')

    SERVER_STATE = {}
    write_state()
