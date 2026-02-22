using System;
using Unravel.Core;

/// <summary>
/// Main Menu script that handles the primary navigation buttons.
/// Uses LobbyUI for centralized navigation instead of direct menu manipulation.
/// Now inherits from BaseMenuController to eliminate code duplication.
/// </summary>
[ScriptSourceFile]
public class MainMenu : BaseMenu
{

	// Menu button elements
	private UIElement playButton;
	private UIElement settingsButton;
	private UIElement exitButton;

	protected override string GetTitleElementId()
	{
		return "main_title";
	}

	protected override void CacheUIElements()
	{
		// Call base implementation to cache title element
		base.CacheUIElements();

		// Cache menu-specific button elements
		playButton = document.GetElementById("play_btn");
		settingsButton = document.GetElementById("options_btn");
		exitButton = document.GetElementById("exit_btn");
	}

	protected override int CountValidElements()
	{
		int count = base.CountValidElements();
		var elements = new UIElement[] { playButton, settingsButton, exitButton };

		foreach (var element in elements)
		{
			if (element?.IsValid() == true) count++;
		}

		return count;
	}

	protected override void RegisterEventHandlers()
	{
		// Register Play button event handlers
		RegisterButtonEvents(playButton, "Play",
			(ev) => OnButtonDown(playButton, ev, "Play"),
			OnPlayButtonClick,
			(ev) => OnButtonHover(playButton, ev, "Play"),
			(ev) => OnButtonLeave(playButton, ev, "Play"),
			(ev) => OnButtonRelease(playButton, ev, "Play"));

		// Register Settings button event handlers
		RegisterButtonEvents(settingsButton, "Settings",
			(ev) => OnButtonDown(settingsButton, ev, "Settings"),
			OnSettingsButtonClick,
			(ev) => OnButtonHover(settingsButton, ev, "Settings"),
			(ev) => OnButtonLeave(settingsButton, ev, "Settings"),
			(ev) => OnButtonRelease(settingsButton, ev, "Settings"));

		// Register Exit button event handlers
		RegisterButtonEvents(exitButton, "Exit",
			(ev) => OnButtonDown(exitButton, ev, "Exit"),
			OnExitButtonClick,
			(ev) => OnButtonHover(exitButton, ev, "Exit"),
			(ev) => OnButtonLeave(exitButton, ev, "Exit"),
			(ev) => OnButtonRelease(exitButton, ev, "Exit"));

		Log.Info("MainMenu event handlers registered successfully");
	}

	// ========== BUTTON CLICK HANDLERS ==========
	// All navigation goes through LobbyUI for consistency

	/// <summary>
	/// Handles the Play button click - starts the game.
	/// </summary>
	private void OnPlayButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Play button clicked - starting game");
		LobbyUI.FindInScene()?.StartGame();
	}

	/// <summary>
	/// Handles the Settings button click - opens settings menu.
	/// </summary>
	private void OnSettingsButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Settings button clicked - opening settings");
		LobbyUI.FindInScene().OpenSettings();
	}

	/// <summary>
	/// Handles the Exit button click - quits the application.
	/// </summary>
	private void OnExitButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Exit button clicked - quitting application");
		Application.Quit();
	}
}
