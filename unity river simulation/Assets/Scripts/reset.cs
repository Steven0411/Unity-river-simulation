using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class reset : MonoBehaviour
{
	public Scene river;
	public void ResetScene(string sceneName)
	{
		SceneManager.LoadScene(sceneName);
		Debug.Log("Loading Scene");
	}
}

