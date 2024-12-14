using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowPhysics : MonoBehaviour
{
	[SerializeField] private Transform target;
	private Rigidbody eb;
	private void Start()
	{
		eb = GetComponent<Rigidbody>();
	}
	private void FixedUpdate()
	{
		eb.velocity = (target.position - transform.position) / Time.fixedDeltaTime;

		Quaternion rotationDifference = target.rotation * Quaternion.Inverse(transform.rotation);
		rotationDifference.ToAngleAxis(out float angleInDegree, out Vector3 rotationAxis);
		Vector3 rotationDifferenceInDegree = angleInDegree * rotationAxis;
		eb.angularVelocity = rotationDifferenceInDegree * Mathf.Deg2Rad / Time.fixedDeltaTime;
	}
}
