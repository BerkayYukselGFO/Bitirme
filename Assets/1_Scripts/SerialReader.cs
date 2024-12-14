using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.IO.Ports;
using System.Globalization;

public class SerialReader : MonoBehaviour
{
    SerialPort serialPort = new SerialPort("COM3", 9600);
    [SerializeField] private List<Quaternion> rawQuaternions;
    [SerializeField] private List<Quaternion> calculatedQuaternions;
    [SerializeField] private List<Quaternion> offsets; // Offset değerlerini sakla
    [SerializeField] private string currentData;
    [SerializeField] private bool calibrateDone;
    [SerializeField] private List<Transform> armTransforms;

    // Kalman filtre için durum ve hata kovaryans matrisleri
    private List<Vector4> kalmanState; // Quaternion (x, y, z, w) için durum vektörü
    private List<Vector4> kalmanErrorCovariance;
    private float processNoise = 0.01f; // Süreç gürültüsü
    private float measurementNoise = 0.1f; // Ölçüm gürültüsü

    void Start()
    {
        rawQuaternions = new List<Quaternion> { Quaternion.identity, Quaternion.identity, Quaternion.identity };
        calculatedQuaternions = new List<Quaternion> { Quaternion.identity, Quaternion.identity, Quaternion.identity };
        offsets = new List<Quaternion> { Quaternion.identity, Quaternion.identity, Quaternion.identity };

        kalmanState = new List<Vector4> { Vector4.zero, Vector4.zero, Vector4.zero };
        kalmanErrorCovariance = new List<Vector4> { Vector4.one*1f, Vector4.one*1f, Vector4.one*1f };

        serialPort.Open();
        StartCoroutine(CalibrateSensors());
    }

    string buffer = ""; // Gelen verileri geçici olarak saklama

    private IEnumerator CalibrateSensors(float overrideTime = 5f)
    {
        yield return new WaitForSeconds(overrideTime); // Sensörün stabilize olmasını bekle

        for (int i = 0; i < rawQuaternions.Count; i++)
        {
            offsets[i] = rawQuaternions[i]; // Offset olarak kaydet
        }
        calibrateDone = true;
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.C))
        {
            StartCoroutine(CalibrateSensors(0f));
        }
        if (serialPort.IsOpen)
        {
            try
            {
                buffer += serialPort.ReadExisting(); // Gelen verileri tamponla
                while (buffer.Contains("\n"))
                {
                    int newlineIndex = buffer.IndexOf("\n");
                    string line = buffer.Substring(0, newlineIndex).Trim();
                    buffer = buffer.Substring(newlineIndex + 1);

                    string[] values = line.Split(','); // Virgülle ayrıştır
                    if (values.Length == 5)
                    { // Değer sayısını kontrol et
                        int channel = int.Parse(values[0]);
                        float q1 = float.Parse(values[1], CultureInfo.InvariantCulture);
                        float q2 = float.Parse(values[2], CultureInfo.InvariantCulture);
                        float q3 = float.Parse(values[3], CultureInfo.InvariantCulture);
                        float q4 = float.Parse(values[4], CultureInfo.InvariantCulture);

                        rawQuaternions[channel] = new Quaternion(-q2, q4, q3, q1);

                        // Kalman filtresini uygula
                        kalmanState[channel] = ApplyKalmanFilter(channel, rawQuaternions[channel]);
                        var filteredQuaternion = new Quaternion(
                            kalmanState[channel].x,
                            kalmanState[channel].y,
                            kalmanState[channel].z,
                            kalmanState[channel].w
                        );

                        calculatedQuaternions[channel] = Quaternion.Inverse(offsets[channel]) * filteredQuaternion;
                        calculatedQuaternions[channel].Normalize();
                    }
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError("Veri işleme hatası: " + ex.Message);
            }
        }

        if (calibrateDone)
        {
            for (int i = 0; i < calculatedQuaternions.Count; i++)
            {
                var target = calculatedQuaternions[i];
                armTransforms[i].transform.localRotation = target;
            }
        }
    }

     private Vector4 ApplyKalmanFilter(int channel, Quaternion measurement)
{
    Vector4 state = kalmanState[channel];
    Vector4 errorCovariance = kalmanErrorCovariance[channel];
    Vector4 measurementVector = new Vector4(measurement.x, measurement.y, measurement.z, measurement.w);

    // Kalman kazancını hesapla
    Vector4 kalmanGain = new Vector4(
        errorCovariance.x / (errorCovariance.x + measurementNoise),
        errorCovariance.y / (errorCovariance.y + measurementNoise),
        errorCovariance.z / (errorCovariance.z + measurementNoise),
        errorCovariance.w / (errorCovariance.w + measurementNoise)
    );

    // Durumu güncelle
    state = state + Vector4.Scale(kalmanGain, (measurementVector - state));

    // Hata kovaryansını güncelle
    errorCovariance = Vector4.Scale(Vector4.one - kalmanGain, errorCovariance) + new Vector4(processNoise, processNoise, processNoise, processNoise);

    // Güncellenen değerleri sakla
    kalmanState[channel] = state;
    kalmanErrorCovariance[channel] = errorCovariance;

    return state;
}

    void OnApplicationQuit()
    {
        serialPort.Close();
    }
}
