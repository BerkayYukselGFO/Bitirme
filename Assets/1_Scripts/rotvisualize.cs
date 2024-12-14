using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class rotvisualize : MonoBehaviour
{

	private void OnDrawGizmos()
	{
		Gizmos.color = Color.red;
		Gizmos.DrawLine(transform.position , transform.position + transform.right*2f);
		Gizmos.color = Color.blue;
		Gizmos.DrawLine(transform.position , transform.position + transform.forward*2f);
		Gizmos.color = Color.green;
		Gizmos.DrawLine(transform.position , transform.position + transform.up*2f);
	}
}
