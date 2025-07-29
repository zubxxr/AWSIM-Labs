using UnityEngine;
using UnityEngine.UI;

namespace AWSIM.Scripts.UI
{
    public class UICameraBridge : MonoBehaviour
    {
        private Canvas cameraOutputCanvas;
        [SerializeField] private float startXOffset;
        [SerializeField] private float startYOffset;
        [SerializeField] private float height;

        private RawImage rawImage;
        private AspectRatioFitter aspectRatioFitter;

        private void Start()
        {
            cameraOutputCanvas = GameObject.FindWithTag("CameraOutputCanvas").GetComponent<Canvas>();
            SetupCameraOutput();
        }

        private void SetupCameraOutput()
        {
            var cameraOutput = new GameObject(
                "CameraOutput",
                typeof(RectTransform),
                typeof(CanvasRenderer),
                typeof(UIInteractionHandler),
                typeof(RawImage),
                typeof(AspectRatioFitter));

            cameraOutput.transform.SetParent(cameraOutputCanvas.transform, false);

            var rectTransform = cameraOutput.GetComponent<RectTransform>();
            rectTransform.anchorMin = Vector2.zero;
            rectTransform.anchorMax = Vector2.zero;
            rectTransform.anchoredPosition = new Vector2(startXOffset, startYOffset);
            rectTransform.sizeDelta = new Vector2(0, height);

            rawImage = cameraOutput.GetComponent<RawImage>();
            aspectRatioFitter = cameraOutput.GetComponent<AspectRatioFitter>();
            aspectRatioFitter.aspectMode = AspectRatioFitter.AspectMode.HeightControlsWidth;

            // Add Vehicle Label
            var labelGO = new GameObject("VehicleLabel", typeof(RectTransform), typeof(CanvasRenderer), typeof(Text));
            labelGO.transform.SetParent(cameraOutput.transform, false);

            var labelText = labelGO.GetComponent<Text>();

            // Traverse hierarchy to find the vehicle GameObject (contains "EgoVehicle")
            Transform current = transform;
            while (current != null)
            {
                if (current.name.Contains("EgoVehicle"))
                {
                    labelText.text = current.name;
                    break;
                }
                current = current.parent;
            }

            // Optional fallback
            if (current == null)
            {
                labelText.text = "Unknown Vehicle";
            }

            labelText.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            labelText.fontSize = 24;
            labelText.color = Color.white;
            labelText.alignment = TextAnchor.UpperLeft;

            var labelRect = labelGO.GetComponent<RectTransform>();
            labelRect.anchorMin = new Vector2(0, 1);
            labelRect.anchorMax = new Vector2(0, 1);
            labelRect.pivot = new Vector2(0, 1);
            labelRect.anchoredPosition = new Vector2(10, -10);
            labelRect.sizeDelta = new Vector2(200, 40);
        }

        public void RenderCameraToUI(CameraSensor.CameraParameters camParams, Texture camRenderTex)
        {
            aspectRatioFitter.aspectRatio = (float)camParams.width / camParams.height;
            rawImage.rectTransform.sizeDelta = new Vector2(height * aspectRatioFitter.aspectRatio, height);
            rawImage.texture = camRenderTex;
        }

        private void OnEnable()
        {
            if (rawImage != null)
                rawImage.gameObject.SetActive(true);
        }

        private void OnDisable()
        {
            if (rawImage != null)
                rawImage.gameObject.SetActive(false);
        }
    }
}
