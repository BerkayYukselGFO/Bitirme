using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BallSpawner : MonoBehaviour
{
	[SerializeField] private GameObject ballPrefab; // Fırlatılacak top prefab'i
	[SerializeField] private Transform spawnPoint; // Topun spawn edildiği nokta
	[SerializeField] private BoxCollider targetArea; // Hedef alan

	[SerializeField] private float launchForce = 10f; // Fırlatma gücü
	[SerializeField] private float launchTime;
	private float timer;
	private void Update()
	{
		timer += Time.deltaTime;
		if(timer > launchTime)
		{
			SpawnAndLaunchBall();
			timer = 0f;
		}
	}

	private void SpawnAndLaunchBall()
	{
		// Topu spawn et
		GameObject ball = Instantiate(ballPrefab, spawnPoint.position, Quaternion.identity);

		// Hedef noktayı seç
		Vector3 targetPoint = GetRandomPointInTargetArea();

		// Fırlatma işlemi
		LaunchBall(ball, targetPoint);
	}

	private Vector3 GetRandomPointInTargetArea()
	{
		// BoxCollider boyutları ve merkezi
		Vector3 center = targetArea.transform.position + targetArea.center;
		Vector3 size = targetArea.size;

		// Hedef noktayı rastgele seç
		float randomX = Random.Range(-size.x / 2, size.x / 2);
		float randomY = Random.Range(-size.y / 2, size.y / 2);
		float randomZ = Random.Range(-size.z / 2, size.z / 2);

		return center + new Vector3(randomX, randomY, randomZ);
	}

	private void LaunchBall(GameObject ball, Vector3 targetPoint)
	{
		// Topun Rigidbody'sini al
		Rigidbody rb = ball.GetComponent<Rigidbody>();

		if (rb == null)
		{
			Debug.LogError("Ball prefab'inde Rigidbody yok!");
			return;
		}

		// Fiziksel hesaplama
		Vector3 direction = targetPoint - spawnPoint.position; // Hedefe olan yön
		float distance = direction.magnitude; // Hedefe olan mesafe
		float gravity = Physics.gravity.magnitude; // Yerçekimi gücü

		// Fırlatma açısı (45 derece optimaldir)
		float angle = 45f * Mathf.Deg2Rad;

		// Fırlatma hızını hesapla
		float velocity = Mathf.Sqrt(distance * gravity / Mathf.Sin(2 * angle));

		// Fırlatma vektörünü hesapla
		Vector3 velocityVector = new Vector3(direction.x, direction.y + Mathf.Tan(angle) * distance, direction.z).normalized * velocity;

		// Topu fırlat
		rb.velocity = velocityVector;
	}
}