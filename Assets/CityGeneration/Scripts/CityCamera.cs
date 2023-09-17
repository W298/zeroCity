using Cinemachine;
using UnityEngine;

public class CityCamera : MonoBehaviour
{
    private CinemachineVirtualCamera _vcam;

    private float _desireZoom;
    
    private void Awake()
    {
        _vcam = GetComponent<CinemachineVirtualCamera>();
        _desireZoom = _vcam.m_Lens.OrthographicSize;
    }

    private void Update()
    {
        if (Input.mouseScrollDelta.y != 0)
        {
            _desireZoom = _vcam.m_Lens.OrthographicSize - Input.mouseScrollDelta.y * 15f;
            _desireZoom = Mathf.Clamp(_desireZoom, 30f, 100f);
        }

        _vcam.m_Lens.OrthographicSize = Mathf.Lerp(_vcam.m_Lens.OrthographicSize, _desireZoom, Time.deltaTime * 10);
    }
}