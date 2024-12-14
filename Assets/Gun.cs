using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Gun : MonoBehaviour
{
	[SerializeField] private Transform firePoint;
	[SerializeField] private float timeThreshold;
	private float timer;
	[SerializeField] private LineRenderer lineRenderer;
	public void Update()
	{
		if (Physics.Raycast(firePoint.transform.position, firePoint.forward, out var hit))
		{
			if (hit.collider.gameObject.layer == LayerMask.NameToLayer("Target"))
			{
				timer += Time.deltaTime;
				if (timer > timeThreshold)
				{
					Destroy(hit.collider.gameObject);
					timer = 0f;
				}
			}
			else
			{
				timer = 0f;
			}
			lineRenderer.positionCount = 2;
			lineRenderer.SetPosition(0, firePoint.transform.position);
			lineRenderer.SetPosition(1, hit.point);
		}
		else
		{
			lineRenderer.positionCount = 2;
			lineRenderer.SetPosition(0, firePoint.transform.position);
			lineRenderer.SetPosition(1, firePoint.transform.position + firePoint.transform.forward * 30f);
		}
	}

}
