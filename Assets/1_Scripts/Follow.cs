using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Follow : MonoBehaviour
{
	[SerializeField] private Transform target;
	private void LateUpdate()
	{
		var targetRot = new Quaternion(-target.transform.rotation.z, -target.transform.rotation.y, -target.transform.rotation.x, target.transform.rotation.w);
		transform.rotation = targetRot;
	}
}
