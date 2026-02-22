using System;
using System.Runtime.CompilerServices;
using Unravel.Core;

/// <summary>
/// Game UI controller that handles in-game UI and menu functionality.
/// Manages ESC key input, game menu, and in-game UI elements.
/// Focused specifically on gameplay scene functionality.
/// </summary>
[ScriptSourceFile]
public class GameUI : ScriptComponent
{
	public Entity GameMenu;
	public Entity SettingsMenu;

	public Entity BackgroundAudioSource;

	public Scene MainMenuScene;

	public bool IsGamePaused { get; private set; } = false;

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

	}

	/// <summary>
	/// Automatically find menu entity references if not manually assigned.
	/// </summary>
	private void FindMenuReferences()
	{
		if (!GameMenu)
		{
			var gameMenuEntity = owner.transform.FindChild("GameMenu", true);
			if (gameMenuEntity) GameMenu = gameMenuEntity;
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
		if (!GameMenu)
		{
			Log.Warning("GameMenu reference not found - ESC key functionality may not work");
		}

		if (!SettingsMenu)
		{
			Log.Warning("SettingsMenu reference not found - settings navigation may not work");
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
		// Handle ESC key for opening/closing game menu
		if (Input.IsPressed(KeyCode.Escape))
		{
			ToggleGameMenu();
		}

		// TODO: Add other game UI logic here
		// Examples:
		// - Health bar updates
		// - Score display
		// - Inventory UI
		// - Chat messages
	}

	/// <summary>
	/// Toggle the game menu on/off and handle pause state.
	/// </summary>
	public void ToggleGameMenu()
	{
		if (!GameMenu)
		{
			Log.Warning("GameMenu reference is not set in GameUI");
			return;
		}

		bool isMenuActive = GameMenu.active;

		if (isMenuActive)
		{
			// Close menu and resume game
			CloseGameMenu();
		}
		else
		{
			// Open menu and pause game
			OpenGameMenu();
		}
	}

	/// <summary>
	/// Open the game menu and pause the game.
	/// </summary>
	public void OpenGameMenu()
	{
		if (IsGamePaused)
		{
			return;
		}
		
		if (GameMenu)
		{
			var gameAudio = Scene.FindEntityByName("GameAudio");
			if (gameAudio)
			{
				var sourceComponents = gameAudio.GetComponentsInChildren<AudioSourceComponent>();
				foreach (var sourceComponent in sourceComponents)
				{
					sourceComponent.Pause();
				}
			}
			GameMenu.SetActive(true);
			BackgroundAudioSource.SetActive(true);
			Time.timeScale = 0f;
			IsGamePaused = true;
			Log.Info("Game menu opened - game paused");
		}
	}

	/// <summary>
	/// Close the game menu and resume the game.
	/// </summary>
	public void CloseGameMenu()
	{
		if (GameMenu)
		{
			var gameAudio = Scene.FindEntityByName("GameAudio");
			if (gameAudio)
			{
				var sourceComponents = gameAudio.GetComponentsInChildren<AudioSourceComponent>();
				foreach (var sourceComponent in sourceComponents)
				{
					sourceComponent.Resume();
				}
			}
			ShowMenu(Entity.Invalid, GameMenu);

			BackgroundAudioSource.SetActive(false);
			Time.timeScale = 1f;
			IsGamePaused = false;
			Log.Info("Game menu closed - game resumed");
		}
	}

	// ========== CENTRALIZED MENU NAVIGATION ==========
	// All menu navigation goes through these methods for consistency

	/// <summary>
	/// Navigate to the main menu (used by quit buttons).
	/// </summary>
	public void GoToMainMenu()
	{
		ResumeGame();
		Log.Info("Navigating to main menu from game");
		if (MainMenuScene != null)
		{
			Scene.LoadScene(MainMenuScene);
		}
		else
		{
			Log.Warning("MainMenuScene reference not set");
		}
	}

	/// <summary>
	/// Resume the game (used by resume button).
	/// </summary>
	public void ResumeGame()
	{
		Log.Info("Resuming game");
		CloseGameMenu();
	}

	/// <summary>
	/// Open settings menu from game menu.
	/// </summary>
	public void OpenSettings()
	{
		Log.Info("Opening settings from game menu");

		// Tell the settings menu which menu to return to (always game menu in this context)
		if (SettingsMenu)
		{
			var settingsController = SettingsMenu.GetComponent<SettingsMenu>();
			if (settingsController != null)
			{
				settingsController.SetPreviousMenu(GameMenu);
			}
		}

		ShowMenu(SettingsMenu, GameMenu);
	}

	/// <summary>
	/// Close settings and return to game menu.
	/// </summary>
	public void CloseSettings()
	{
		Log.Info("Closing settings, returning to game menu");
		ShowMenu(GameMenu, SettingsMenu);
	}

	/// <summary>
	/// Quit the application (used by exit buttons).
	/// </summary>
	public void QuitApplication()
	{
		Log.Info("Quitting application");
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

		Log.Info($"Menu transition: {(menuToHide ? menuToHide.name : "null")} -> {(menuToShow ? menuToShow.name : "null")}");
	}

	/// <summary>
	/// Static helper method to find the GameUI controller in the current scene.
	/// Can be used by menus to find the game controller.
	/// </summary>
	public static GameUI FindInScene()
	{
		var gameUIEntity = Scene.FindEntityByName("GameUI");
		if (gameUIEntity)
		{
			return gameUIEntity.GetComponent<GameUI>();
		}
		return null;
	}

	/// <summary>
	/// For more functions <see cref="ScriptComponent"/>.
	/// </summary>
}