using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class move : MonoBehaviour
{
    NavAgent navAgent;
    public GameObject end;
    void Start()
    {
        navAgent = GetComponent<NavAgent>();
        StartCoroutine(s());
    }
    IEnumerator s()
    {
        yield return new WaitForEndOfFrame();
        navAgent.SetDestination(end.transform.position);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
