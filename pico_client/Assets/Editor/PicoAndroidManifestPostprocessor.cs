using System.IO;
using System.Xml;
using UnityEditor.Android;

public sealed class PicoAndroidManifestPostprocessor : IPostGenerateGradleAndroidProject
{
    private const string AndroidNamespace = "http://schemas.android.com/apk/res/android";

    public int callbackOrder => 100;

    public void OnPostGenerateGradleAndroidProject(string path)
    {
        string manifestPath = Path.Combine(path, "src", "main", "AndroidManifest.xml");
        var document = new XmlDocument();
        document.Load(manifestPath);
        XmlElement manifest = document.DocumentElement;
        AddPermission(document, manifest, "android.permission.ACCESS_WIFI_STATE");
        AddPermission(document, manifest, "android.permission.CHANGE_WIFI_STATE");
        AddPermission(document, manifest, "android.permission.WAKE_LOCK");
        document.Save(manifestPath);
    }

    private static void AddPermission(XmlDocument document, XmlElement manifest, string name)
    {
        foreach (XmlNode node in manifest.SelectNodes("uses-permission"))
        {
            if (node.Attributes?["name", AndroidNamespace]?.Value == name)
            {
                return;
            }
        }
        XmlElement permission = document.CreateElement("uses-permission");
        permission.SetAttribute("name", AndroidNamespace, name);
        manifest.PrependChild(permission);
    }
}
