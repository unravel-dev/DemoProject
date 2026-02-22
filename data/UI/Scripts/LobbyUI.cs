using System;
using System.Runtime.CompilerServices;
using Unravel.Core;

/// <summary>
/// Lobby UI controller that handles main menu navigation and scene transitions.
/// Manages the main menu, settings menu, and transitions to the game scene.
/// </summary>
[ScriptSourceFile]

public class LobbyUI : ScriptComponent
{
	public Entity MainMenu;
	public Entity SettingsMenu;

	public Scene GameScene;

	public Entity BackgroundAudioSource;

	/// <summary>
	/// OnCreate is called when the script is first loaded, or when an object it is attached to is instantiated
	/// It only gets called once on each script, and only after other objects are initialised.
	/// This means that it is safe to create references to other game objects and components in OnCreate.
	/// </summary>
	public override void OnCreate()
	{
		// No singleton setup needed - menus will find this via Scene.FindEntityByName
	}

	/// <summary>
	/// Start is called once, before any Update methods and after OnCreate.
	/// It works in much the same way as OnCreate, with a few key differences.
	/// Unlike OnCreate, Start will not be called if the script is disabled.
	/// </summary>
	public override void OnStart()
	{
		// Auto-find menu references if not assigned
		FindMenuReferences();

		// Validate required references
		ValidateReferences();

		BackgroundAudioSource.SetActive(true);
	}

	/// <summary>
	/// Automatically find menu entity references if not manually assigned.
	/// </summary>
	private void FindMenuReferences()
	{
		if (!MainMenu)
		{
			var mainMenuEntity = owner.transform.FindChild("MainMenu", true);
			if (mainMenuEntity) MainMenu = mainMenuEntity;
		}

		if (!SettingsMenu)
		{
			var settingsMenuEntity = owner.transform.FindChild("SettingsMenu", true);
			if (settingsMenuEntity) SettingsMenu = settingsMenuEntity;
		}

		if (!BackgroundAudioSource)
		{
			var backgroundAudioSourceEntity = owner.transform.FindChild("BackgroundAudioSource", true);
			if (backgroundAudioSourceEntity) BackgroundAudioSource = backgroundAudioSourceEntity;
		}
	}

	/// <summary>
	/// Validate that required references are set.
	/// </summary>
	private void ValidateReferences()
	{
		if (!MainMenu)
		{
			Log.Warning("MainMenu reference not found - main menu functionality may not work");
		}

		if (!SettingsMenu)
		{
			Log.Warning("SettingsMenu reference not found - settings navigation may not work");
		}

		if (GameScene == null)
		{
			Log.Warning("GameScene reference not set - play button may not work");
		}

		if (!BackgroundAudioSource)
		{
			Log.Warning("BackgroundAudioSource reference not found - background audio may not work");
		}
	}

	public override void OnEnable()
	{
	}

	public override void OnDisable()
	{
	}

	public override void OnUpdate()
	{
		// Lobby UI doesn't need continuous updates
		// All interactions are event-driven through menu callbacks
	}

	// ========== CENTRALIZED LOBBY NAVIGATION ==========
	// All lobby menu navigation goes through these methods for consistency

	/// <summary>
	/// Start the game (used by play button).
	/// </summary>
	public void StartGame()
	{
		Log.Info("Starting game from lobby");
		if (GameScene != null)
		{
			Scene.LoadScene(GameScene);
		}
		else
		{
			Log.Warning("GameScene reference not set");
		}
	}

	/// <summary>
	/// Open settings menu from main menu.
	/// </summary>
	public void OpenSettings()
	{
		Log.Info("Opening settings from main menu");

		// Tell the settings menu which menu to return to
		if (SettingsMenu)
		{
			var settingsController = SettingsMenu.GetComponent<SettingsMenu>();
			if (settingsController != null)
			{
				settingsController.SetPreviousMenu(MainMenu);
			}
		}

		ShowMenu(SettingsMenu, MainMenu);
	}

	/// <summary>
	/// Close settings and return to main menu.
	/// </summary>
	public void CloseSettings()
	{
		Log.Info("Closing settings, returning to main menu");
		ShowMenu(MainMenu, SettingsMenu);
	}

	/// <summary>
	/// Quit the application (used by exit button).
	/// </summary>
	public void QuitApplication()
	{
		Log.Info("Quitting application from lobby");
		Application.Quit();
	}

	/// <summary>
	/// Show a specific menu and hide others (internal helper method).
	/// </summary>
	/// <param name="menuToShow">The menu entity to show</param>
	/// <param name="menuToHide">The menu entity to hide</param>
	private void ShowMenu(Entity menuToShow, Entity menuToHide)
	{
		if (menuToHide)
		{
			menuToHide.SetActive(false);
		}

		if (menuToShow)
		{
			menuToShow.SetActive(true);
		}

		Log.Info($"Lobby menu transition: {(menuToHide ? menuToHide.name : "null")} -> {(menuToShow ? menuToShow.name : "null")}");
	}

	/// <summary>
	/// Static helper method to find the LobbyUI controller in the current scene.
	/// Can be used by menus to find the lobby controller.
	/// </summary>
	public static LobbyUI FindInScene()
	{
		var lobbyUIEntity = Scene.FindEntityByName("LobbyUI");
		if (lobbyUIEntity)
		{
			return lobbyUIEntity.GetComponent<LobbyUI>();
		}
		return null;
	}

	/// <summary>
	/// For more functions <see cref="ScriptComponent"/>.
	/// </summary>
}
