import flask

def create_response(value):
    response = flask.jsonify(value)
    response.headers.add("Access-Control-Allow-Origin", "*")
    return response