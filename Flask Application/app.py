from flask import Flask, jsonify, render_template, request
from flask_sqlalchemy import SQLAlchemy
from sqlalchemy import func



app = Flask(__name__)


app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///site.db' 
db = SQLAlchemy(app)

# Define your data model
class SensorData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    sensor_id = db.Column(db.Integer)
    temperature = db.Column(db.Float)
    humidity = db.Column(db.Float)
    pressure = db.Column(db.Float)
    voc_gas = db.Column(db.Float)
    light = db.Column(db.Float)
    movement_counter = db.Column(db.Integer)

# Create the database tables
with app.app_context():
    print("Before creating tables")

    db.create_all()
    print("After creating tables")


@app.route('/')
def index():
    # Fetch the latest sensor reading for each sensor
    latest_readings = db.session.query(
        SensorData.sensor_id,
        func.max(SensorData.id).label('max_id')
    ).group_by(SensorData.sensor_id).subquery()

    sensor_data = db.session.query(SensorData).join(
        latest_readings,
        (SensorData.id == latest_readings.c.max_id)
    ).all()

    print(sensor_data)

    return render_template('index.html', sensor_data=sensor_data)

@app.route('/update', methods=['POST'])
def update_data():
    print("Before database operation")
    # Receive data from IoT sensor nodes
    data = request.get_json()

    print(f"Received data: {data}")

    # Check if required fields are present in the JSON payload
    required_fields = ['sensor_id', 'temperature', 'humidity', 'pressure', 'voc_gas', 'light', 'movement_counter']
    if not all(field in data for field in required_fields):
        print("Invalid data format. Make sure all required fields are present.")

        return jsonify({'error': 'Invalid data format. Make sure all required fields are present.'}), 400

    # Check if a record for the sensor already exists in the database
    existing_data = SensorData.query.filter_by(sensor_id=data['sensor_id']).first()

    if existing_data:
        # Update the existing record
        existing_data.temperature = data['temperature']
        existing_data.humidity = data['humidity']
        existing_data.pressure = data['pressure']
        existing_data.voc_gas = data['voc_gas']
        existing_data.light = data['light']
        existing_data.movement_counter = data['movement_counter']
    else:
        # Create a new entry in the database if the sensor record doesn't exist
        new_data = SensorData(
            sensor_id=data['sensor_id'],
            temperature=data['temperature'],
            humidity=data['humidity'],
            pressure=data['pressure'],
            voc_gas=data['voc_gas'],
            light=data['light'],
            movement_counter=data['movement_counter']
        )
        db.session.add(new_data)

    # Commit changes to the database
    db.session.commit()

    print("After database operation")

    return jsonify({'message': 'Data received and stored successfully.'})

if __name__ == '__main__':
    app.run(debug=True)