using UnityEngine;

public class PointCloudReader : MonoBehaviour
{
    [SerializeField] private string fileName;
    [SerializeField] private Material material;

    void Awake()
    {
        string path = Application.dataPath + "/Resources/" + fileName + ".txt";

        if (!System.IO.File.Exists(path))
        {
            Debug.LogError("File not found: " + path);
            return;
        }

        string[] lines = System.IO.File.ReadAllLines(path);

        if (lines.Length < 1)
        {
            Debug.LogError("File content is empty.");
            return;
        }

        if (!int.TryParse(lines[0], out int n))
        {
            Debug.LogError("Invalid integer value for the number of points.");
            return;
        }

        for (int i = 1; i <= n; i++)
        {
            if (i >= lines.Length)
            {
                Debug.LogError("File content is shorter than expected.");
                break;
            }

            string[] values = lines[i].Split(' ');

            if (values.Length < 3 || !float.TryParse(values[0], out float x) || !float.TryParse(values[1], out float y) || !float.TryParse(values[2], out float z))
            {
                Debug.LogError("Invalid values: " + lines[i]);
                continue;
            }

            CreateSphere(new Vector3(x, y, z), i);
        }
    }

    private void CreateSphere(Vector3 position, int index)
    {
        GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Cube);
        sphere.transform.position = position;
        sphere.GetComponent<Renderer>().material = material;
        sphere.name = "P " + index;
        sphere.transform.parent = transform;
    }
}

