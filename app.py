from flask import Flask, request, jsonify, render_template
import mysql.connector
from datetime import datetime

app = Flask(__name__)

# MySQL Configuration
db = mysql.connector.connect(
    host="localhost",
    user="root",
    password="password",      # Change this
    database="sensor_data"    # Make sure this DB and table exist
)
cursor = db.cursor()

latest_data = {}

@app.route('/')
def index():
    return render_template('index.html', data=latest_data)

@app.route('/sensor', methods=['POST'])
def receive_data():
    global latest_data
    try:
        data = request.get_json()
        print("Raw JSON received from ESP:", data)

        temp = float(data.get("temperature"))
        hum = float(data.get("humidity"))
        press = float(data.get("pressure"))
        uv = float(data.get("uv_index"))
        alt = float(data.get("altitude"))
        now = datetime.now()

        print(f"Parsed -> Temp: {temp}, Humidity: {hum}, Pressure: {press}, UV: {uv}, Altitude: {alt}")

        insert_query = """
            INSERT INTO data_atmosphere (Temperature, Humidity, Pressure, UV_index, Altitude, entry_time)
            VALUES (%s, %s, %s, %s, %s, %s)
        """
        values = (temp, hum, press, uv, alt, now)
        cursor.execute(insert_query, values)
        db.commit()

        latest_data = {
            "Temperature": temp,
            "Humidity": hum,
            "Pressure": press,
            "UV_index": uv,
            "Altitude": alt,
            "entry_time": now.strftime("%Y-%m-%d %H:%M:%S")
        }

        return jsonify({"status": "success"}), 200

    except Exception as e:
        print("Error while processing data:", e)
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
