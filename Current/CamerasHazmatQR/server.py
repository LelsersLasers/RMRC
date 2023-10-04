from flask import Flask, render_template, Response
import json
import time


JSON_FILE = "state.json"

STATE = {
    "frame": ""
}

def read_state():
	global STATE
	try:
		with open(JSON_FILE, "r") as f:
			STATE = json.load(f)
	except:
		time.sleep(0.1)
		read_state()


app=Flask(__name__)

@app.route('/')
def index():
	return render_template('index.html')

@app.route('/get')
def get():
	read_state()
	return Response(json.dumps(STATE), mimetype='application/json')
