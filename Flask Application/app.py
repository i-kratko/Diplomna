#TODO NEEDS TO SUPPROT TWO DIFFERENT NODE TYPES

from flask import Flask, render_template, request, jsonify
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

app = Flask(__name__)

app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///site.db'  # Example: SQLite database
db = SQLAlchemy(app)

class SensorData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    temperature = db.Column(db.Float, nullable=False)
    humidity = db.Column(db.Float, nullable=False)
    timestamp = db.Column(db.DateTime, default=datetime.utcnow)

    def __repr__(self):
        return f"SensorData(temperature={self.temperature}, humidity={self.humidity}, timestamp={self.timestamp})"

@app.route('/')
def index():
    # Render your main page with graphs
    return render_template('index.html')

@app.route('/sensor-data', methods=['POST'])
def receive_sensor_data():
    data = request.get_json()
    #multiple sensor node logic TODO
    new_data = SensorData(temperature=data['temperature'], humidity=data['humidity'])
    db.session.add(new_data)
    db.session.commit()
    return jsonify({'message': 'Data received successfully'})

if __name__ == '__main__':
    app.run(host='0.0.0.0',debug=True)