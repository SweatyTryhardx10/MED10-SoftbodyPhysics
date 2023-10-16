using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEditor.Overlays;
using UnityEditor.Toolbars;
using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;
using UnityEngine.UIElements;


[Overlay(typeof(SceneView), "Play Hook Overlay", true)]
public class PlayHookOverlay : ToolbarOverlay
{
    // This is how you do it.
    PlayHookOverlay() : base(InitOnPlay.ID, FastPlaymode.ID) { }


    [EditorToolbarElement(ID, typeof(SceneView))]
    private class InitOnPlay : EditorToolbarToggle
    {
        public const string ID = "LoadHook";

        public static bool LoadSceneOnPlay
        {
            get => EditorPrefs.GetInt("loadOnPlay") == 1 ? true : false;
            set
            {
                if (value)
                    EditorPrefs.SetInt("loadOnPlay", 1);
                else
                    EditorPrefs.SetInt("loadOnPlay", 0);
            }
        }

        public InitOnPlay()
        {
            text = " INIT_SCENE";

            tooltip = "Loads the INIT_SCENE when entering play mode.";
            value = EditorPrefs.GetInt("loadOnPlay") == 1 ? true : false;

            icon = (Texture2D)EditorGUIUtility.IconContent("UnityLogo").image;

            // Register callback
            RegisterCallback<ChangeEvent<bool>>(ToggleSceneLoadFunction);
        }

        private void ToggleSceneLoadFunction(ChangeEvent<bool> evt) { LoadSceneOnPlay = evt.newValue; }

        [InitializeOnLoadMethod]
        public static void Init()
        {
            EditorApplication.playModeStateChanged += LoadSceneOnPlayMode;
        }

        private static void LoadSceneOnPlayMode(PlayModeStateChange playState)
        {
            if (playState == PlayModeStateChange.EnteredPlayMode)
            {
                if (!(EditorPrefs.GetInt("loadOnPlay") == 1))
                    return;

                if (EditorSceneManager.GetSceneAt(0) != EditorSceneManager.GetSceneByBuildIndex(0))
                {
                    string selectedSceneName = EditorSceneManager.GetSceneAt(0).name;
                    int selectedSceneBuildIndex = EditorSceneManager.GetSceneAt(0).buildIndex;

                    // Load INIT_SCENE and then the selected scene additively
                    AsyncOperation initOp = EditorSceneManager.LoadSceneAsync(0, LoadSceneMode.Single);
                }
            }
        }
    }

    [EditorToolbarElement(ID, typeof(SceneView))]
    private class FastPlaymode : EditorToolbarToggle
    {
        public const string ID = "FastPlay";

        public static bool PlayFast
        {
            get => EditorPrefs.GetInt("playFast") == 1 ? true : false;
            set
            {
                if (value)
                    EditorPrefs.SetInt("playFast", 1);
                else
                    EditorPrefs.SetInt("playFast", 0);
            }
        }

        FastPlaymode()
        {
            text = " FAST PLAY";

            tooltip = "Toggles playmode domain reload.";
            value = EditorPrefs.GetInt("playFast") == 1 ? true : false;

            icon = (Texture2D)EditorGUIUtility.IconContent("d_PlayButton").image;

            // Register callback
            RegisterCallback<ChangeEvent<bool>>(ToggleFastPlaymode);
        }

        private void ToggleFastPlaymode(ChangeEvent<bool> evt)
        {
            PlayFast = evt.newValue;

            EditorSettings.enterPlayModeOptionsEnabled = PlayFast;
            
            if (PlayFast)
                EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload | EnterPlayModeOptions.DisableSceneReload;
            else
                EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.None;
        }
    }
}