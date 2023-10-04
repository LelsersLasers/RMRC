from flask import Flask, render_template, Response, jsonify
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

@app.route('/get', methods=['GET'])
def get():
	global STATE

	read_state()
	response = jsonify(STATE)
	response.headers.add('Access-Control-Allow-Origin', '*')
	return response

if __name__ == "__main__":
	read_state()
	app.run(debug=True, port=5000, host='0.0.0.0')
