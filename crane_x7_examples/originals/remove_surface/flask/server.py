from flask import Flask, render_template, request
import os
from flask_socketio import SocketIO, emit, send


app = Flask(__name__)
app.config['JSON_AS_ASCII'] = False
socketio = SocketIO(app)

@app.route('/')
def home():
    return render_template("home.html")

@app.route('/run_experiment', methods=['POST'])
def run_experiment():

    script = request.files['script']
    upload_dir = os.getcwd()
    print(upload_dir)
    script.save(os.path.join(upload_dir + '/tmp/', script.filename))

    os.system("python3 " + upload_dir + '/tmp/' + script.filename + ' &')


    #os.remove(upload_dir + '/tmp/' + script.filename)

    return render_template('status.html')

@app.route('/stop_experiment', methods=['GET'])
def stop_experiment():
    try:
        os.system("rosnode kill randomMove")
    except:
        pass
    
    return render_template('home.html')

@socketio.on('connect')
def test_connect(auth):
    #emit('my response', {'data': 'Connected'})
    print('connected')

@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')
    try:
        os.system("rosnode kill randomMove")
    except:
        pass
    


if __name__ == "__main__":
    #app.run(debug=True)
    socketio.run(app)