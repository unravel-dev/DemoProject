using System;
using Unravel.Core;

/// <summary>
/// Game Menu script that handles in-game navigation buttons.
/// Uses GameUI for centralized navigation instead of direct menu manipulation.
/// Now inherits from BaseMenuController to eliminate code duplication.
/// </summary>
[ScriptSourceFile]
public class GameMenu : BaseMenu
{

	// Menu button elements
	private UIElement resumeButton;
	private UIElement settingsButton;
	private UIElement quitButton;

	protected override string GetTitleElementId()
	{
		return "game_title";
	}

	protected override void CacheUIElements()
	{
		// Call base implementation to cache title element
		base.CacheUIElements();

		// Cache menu-specific button elements
		resumeButton = document.GetElementById("resume_btn");
		settingsButton = document.GetElementById("settings_btn");
		quitButton = document.GetElementById("quit_btn");
	}

	protected override int CountValidElements()
	{
		int count = base.CountValidElements();
		var elements = new UIElement[] { resumeButton, settingsButton, quitButton };

		foreach (var element in elements)
		{
			if (element?.IsValid() == true) count++;
		}

		return count;
	}

	protected override void RegisterEventHandlers()
	{
		// Register Resume button event handlers
		RegisterButtonEvents(resumeButton, "Resume",
			(ev) => OnButtonDown(resumeButton, ev, "Resume"),
			OnResumeButtonClick,
			(ev) => OnButtonHover(resumeButton, ev, "Resume"),
			(ev) => OnButtonLeave(resumeButton, ev, "Resume"),
			(ev) => OnButtonRelease(resumeButton, ev, "Resume"));

		// Register Settings button event handlers
		RegisterButtonEvents(settingsButton, "Settings",
			(ev) => OnButtonDown(settingsButton, ev, "Settings"),
			OnSettingsButtonClick,
			(ev) => OnButtonHover(settingsButton, ev, "Settings"),
			(ev) => OnButtonLeave(settingsButton, ev, "Settings"),
			(ev) => OnButtonRelease(settingsButton, ev, "Settings"));

		// Register Quit button event handlers
		RegisterButtonEvents(quitButton, "Quit",
			(ev) => OnButtonDown(quitButton, ev, "Quit"),
			OnQuitButtonClick,
			(ev) => OnButtonHover(quitButton, ev, "Quit"),
			(ev) => OnButtonLeave(quitButton, ev, "Quit"),
			(ev) => OnButtonRelease(quitButton, ev, "Quit"));

		Log.Info("GameMenu event handlers registered successfully");
	}

	// ========== BUTTON CLICK HANDLERS ==========
	// All navigation goes through GameUI for consistency

	/// <summary>
	/// Handles the Resume button click - resumes the game via GameUI.
	/// </summary>
	private void OnResumeButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Resume button clicked");
		GameUI.FindInScene()?.ResumeGame();
	}

	/// <summary>
	/// Handles the Settings button click - opens settings menu via GameUI.
	/// </summary>
	private void OnSettingsButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Settings button clicked");
		GameUI.FindInScene()?.OpenSettings();
	}

	/// <summary>
	/// Handles the Quit button click - returns to main menu via GameUI.
	/// </summary>
	private void OnQuitButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Quit button clicked");
		GameUI.FindInScene()?.GoToMainMenu();
	}
}
