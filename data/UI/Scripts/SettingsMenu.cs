using System;
using Unravel.Core;

/// <summary>
/// Settings Menu script that handles game settings configuration.
/// Uses GameUI for centralized navigation instead of direct menu manipulation.
/// Now inherits from BaseMenuController to eliminate code duplication.
/// 
/// DYNAMIC GRAPHICS PRESETS USAGE:
/// To replace the HTML dropdown options with custom presets from code:
/// 
/// Example 1 - Basic usage:
/// var settingsMenu = settingsEntity.GetComponent<SettingsMenu>();
/// var customPresets = new SettingsMenu.GraphicsPreset[]
/// {
///     new SettingsMenu.GraphicsPreset("potato", "Potato", "Minimum settings for very low-end hardware"),
///     new SettingsMenu.GraphicsPreset("low", "Low", "Optimized for performance"),
///     new SettingsMenu.GraphicsPreset("medium", "Medium", "Balanced quality and performance"),
///     new SettingsMenu.GraphicsPreset("high", "High", "Enhanced visual quality"),
///     new SettingsMenu.GraphicsPreset("ultra", "Ultra", "Maximum visual fidelity"),
///     new SettingsMenu.GraphicsPreset("extreme", "Extreme", "Beyond ultra - for high-end systems only")
/// };
/// settingsMenu.SetGraphicsPresets(customPresets);
/// 
/// Example 2 - Hardware-based presets:
/// var presets = GetPresetsBasedOnHardware(); // Your custom logic
/// settingsMenu.SetGraphicsPresets(presets);
/// 
/// If no custom presets are set, the HTML defaults (Low, Medium, High, Ultra) will be used.
/// </summary>
[ScriptSourceFile]
public class SettingsMenu : BaseMenu
{
	
	// Track which menu we came from for proper back navigation
	private Entity previousMenu;
	
	/// <summary>
	/// Set the previous menu for proper back navigation.
	/// Called by GameUI when opening settings.
	/// </summary>
	/// <param name="fromMenu">The menu we came from</param>
	public void SetPreviousMenu(Entity fromMenu)
	{
		previousMenu = fromMenu;
	}
	
	/// <summary>
	/// Set custom graphics presets that will replace the HTML defaults.
	/// Call this method before the menu is displayed to populate the dropdown dynamically.
	/// </summary>
	/// <param name="presets">Array of graphics presets to display</param>
	public void SetGraphicsPresets(GraphicsPreset[] presets)
	{
		if (presets != null && presets.Length > 0)
		{
			graphicsPresets = presets;
			hasCustomPresets = true;
			
			// If the dropdown is already initialized, populate it immediately
			if (graphicsQualityDropdown != null)
			{
				PopulateGraphicsDropdown();
			}
			
			Log.Info($"Set {presets.Length} custom graphics presets");
		}
		else
		{
			Log.Warning("Invalid graphics presets provided");
		}
	}
	
	/// <summary>
	/// Get the current graphics presets (either custom or default HTML ones).
	/// </summary>
	/// <returns>Array of current graphics presets</returns>
	public GraphicsPreset[] GetGraphicsPresets()
	{
		if (hasCustomPresets && graphicsPresets != null)
		{
			return graphicsPresets;
		}
		
		// Return default presets that match the HTML
		return GetDefaultGraphicsPresets();
	}
	
	/// <summary>
	/// Get the default graphics presets that match the HTML options.
	/// </summary>
	/// <returns>Array of default graphics presets</returns>
	private GraphicsPreset[] GetDefaultGraphicsPresets()
	{
		return new GraphicsPreset[]
		{
			new GraphicsPreset("low", "Low", "Optimized for performance"),
			new GraphicsPreset("medium", "Medium", "Balanced quality and performance"),
			new GraphicsPreset("high", "High", "Enhanced visual quality"),
			new GraphicsPreset("ultra", "Ultra", "Maximum visual fidelity")
		};
	}

	// Settings control elements
	private UIElement volumeSlider;
	private UIElement volumeLabel;
	private UIElement graphicsQualityDropdown;
	private UIElement graphicsQualityLabel;
	private UIElement backButton;
	private UIElement applyButton;
	
	// Settings values
	private float currentVolume = 1.0f;
	private string currentGraphicsQuality = "high";
	
	// Graphics preset data structure
	public class GraphicsPreset
	{
		public string Value { get; set; }
		public string DisplayName { get; set; }
		public string Description { get; set; }
		
		public GraphicsPreset(string value, string displayName, string description = "")
		{
			Value = value;
			DisplayName = displayName;
			Description = description;
		}
	}
	
	// Available graphics presets - can be modified from external code
	private GraphicsPreset[] graphicsPresets = null;
	private bool hasCustomPresets = false;

	protected override string GetTitleElementId()
	{
		return "settings_title";
	}

	protected override void CacheUIElements()
	{
		// Call base implementation to cache title element
		base.CacheUIElements();
		
		// Cache settings-specific control elements
		volumeSlider = document.GetElementById("volume_slider");
		volumeLabel = document.GetElementById("volume_label");
		graphicsQualityDropdown = document.GetElementById("graphics_quality_dropdown");
		graphicsQualityLabel = document.GetElementById("graphics_quality_label");
		backButton = document.GetElementById("back_btn");
		applyButton = document.GetElementById("apply_btn");
	}
	
	protected override int CountValidElements()
	{
		int count = base.CountValidElements();
		var elements = new UIElement[] { volumeSlider, volumeLabel, graphicsQualityDropdown, graphicsQualityLabel, backButton, applyButton };
		
		foreach (var element in elements)
		{
			if (element?.IsValid() == true) count++;
		}
		
		return count;
	}

	protected override void SetupInitialUI()
	{
		// Call base implementation
		base.SetupInitialUI();
		
		// Set initial volume slider value
		if (volumeSlider != null)
		{
			volumeSlider.SetAttribute("value", currentVolume.ToString("F2"));
		}
		
		// Populate graphics dropdown if custom presets are available
		if (hasCustomPresets && graphicsQualityDropdown != null)
		{
			PopulateGraphicsDropdown();
		}
		
		// Set initial graphics quality dropdown value
		if (graphicsQualityDropdown != null)
		{
			graphicsQualityDropdown.SetAttribute("value", currentGraphicsQuality);
		}
		
		// Update labels
		UpdateVolumeLabel();
		UpdateGraphicsQualityLabel();
	}

	protected override void RegisterEventHandlers()
	{
		// Register Volume Slider event handlers
		if (volumeSlider != null)
		{
			volumeSlider.RegisterCallback<UISliderEvent>("change", OnVolumeSliderChange);
			volumeSlider.RegisterCallback<UISliderEvent>("input", OnVolumeSliderInput);
			Log.Info("Volume slider event handlers registered");
		}
		else
		{
			Log.Warning("Volume slider not found or invalid");
		}
		
		// Register Graphics Quality Dropdown event handlers
		if (graphicsQualityDropdown != null)
		{
			graphicsQualityDropdown.RegisterCallback<UIChangeEvent>("change", OnGraphicsQualityChange);
			Log.Info("Graphics quality dropdown event handlers registered");
		}
		else
		{
			Log.Warning("Graphics quality dropdown not found or invalid");
		}
		
		// Register Back button event handlers
		RegisterButtonEvents(backButton, "Back",
			(ev) => OnButtonDown(backButton, ev, "Back"),
			OnBackButtonClick,
			(ev) => OnButtonHover(backButton, ev, "Back"),
			(ev) => OnButtonLeave(backButton, ev, "Back"),
			(ev) => OnButtonRelease(backButton, ev, "Back"));
		
		// Register Apply button event handlers
		RegisterButtonEvents(applyButton, "Apply",
			(ev) => OnButtonDown(applyButton, ev, "Apply"),
			OnApplyButtonClick,
			(ev) => OnButtonHover(applyButton, ev, "Apply"),
			(ev) => OnButtonLeave(applyButton, ev, "Apply"),
			(ev) => OnButtonRelease(applyButton, ev, "Apply"));
		
		Log.Info("SettingsMenu event handlers registered successfully");
	}

	// ========== SLIDER EVENT HANDLERS ==========
	// Volume slider event handlers

	/// <summary>
	/// Handles the volume slider input event (real-time updates while dragging).
	/// </summary>
	/// <param name="ev">Input event containing slider value</param>
	private void OnVolumeSliderInput(UISliderEvent ev)
	{
		currentVolume = Math.Max(0.0f, Math.Min(1.0f, ev.value));
		UpdateVolumeLabel();
		ApplyVolumeSettings();
		Log.Info($"Volume slider input: {currentVolume:F2}");
	}

	/// <summary>
	/// Handles the volume slider change event (final value when released).
	/// </summary>
	/// <param name="ev">Input event containing final slider value</param>
	private void OnVolumeSliderChange(UISliderEvent ev)
	{
		currentVolume = Math.Max(0.0f, Math.Min(1.0f, ev.value));
		UpdateVolumeLabel();
		ApplyVolumeSettings();
		Log.Info($"Volume slider changed to: {currentVolume:F2}");
	}

	// ========== DROPDOWN EVENT HANDLERS ==========
	// Graphics quality dropdown event handlers

	/// <summary>
	/// Handles the graphics quality dropdown change event.
	/// </summary>
	/// <param name="ev">Change event containing selected value</param>
	private void OnGraphicsQualityChange(UIChangeEvent ev)
	{
		currentGraphicsQuality = ev.value;
		UpdateGraphicsQualityLabel();
		ApplyGraphicsQualitySettings();
		Log.Info($"Graphics quality changed to: {currentGraphicsQuality}");
	}

	// ========== BUTTON CLICK HANDLERS ==========
	// Only the specific business logic for each button

	/// <summary>
	/// Handles the Back button click - returns to previous menu.
	/// </summary>
	private void OnBackButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Back button clicked - returning to previous menu");
		if (previousMenu)
		{
			previousMenu.SetActive(true);
			owner.SetActive(false);
		}
		else
		{
			Log.Warning("No previous menu set - cannot navigate back");
		}
	}
	
	/// <summary>
	/// Handles the Apply button click - applies all settings.
	/// </summary>
	private void OnApplyButtonClick(UIPointerEvent ev)
	{
		Log.Info($"Apply button clicked - applying settings");
		ApplyAllSettings();
	}

	// ========== SETTINGS LOGIC ==========
	// Settings application and management

	/// <summary>
	/// Populates the graphics quality dropdown with custom presets, replacing HTML defaults.
	/// </summary>
	private void PopulateGraphicsDropdown()
	{
		if (graphicsQualityDropdown == null)
		{
			Log.Warning("Graphics quality dropdown not available for population");
			return;
		}
		
		if (!hasCustomPresets || graphicsPresets == null)
		{
			Log.Info("No custom graphics presets available, keeping HTML defaults");
			return;
		}
		
		try
		{
			// Clear existing options
			graphicsQualityDropdown.InnerRml = "";
			
			// Add new options from presets
			string optionsHtml = "";
			foreach (var preset in graphicsPresets)
			{
				string selectedAttribute = preset.Value == currentGraphicsQuality ? " selected" : "";
				string titleAttribute = !string.IsNullOrEmpty(preset.Description) ? $" title=\"{preset.Description}\"" : "";
				
				optionsHtml += $"<option value=\"{preset.Value}\"{selectedAttribute}{titleAttribute}>{preset.DisplayName}</option>";
			}
			
			graphicsQualityDropdown.InnerRml = optionsHtml;
			
			Log.Info($"Populated graphics dropdown with {graphicsPresets.Length} custom presets");
		}
		catch (Exception ex)
		{
			Log.Error($"Failed to populate graphics dropdown: {ex.Message}");
		}
	}

	/// <summary>
	/// Updates the volume label to show current volume percentage.
	/// </summary>
	private void UpdateVolumeLabel()
	{
		if (volumeLabel != null)
		{
			int volumePercent = (int)(currentVolume * 100);
			volumeLabel.InnerRml = $"Volume: {volumePercent}%";
		}
	}

	/// <summary>
	/// Updates the graphics quality label to show current selection.
	/// </summary>
	private void UpdateGraphicsQualityLabel()
	{
		if (graphicsQualityLabel != null)
		{
			string displayName = GetGraphicsQualityDisplayName(currentGraphicsQuality);
			graphicsQualityLabel.InnerRml = $"Graphics Quality: {displayName}";
		}
	}
	
	/// <summary>
	/// Gets the display name for a graphics quality value, checking both custom presets and defaults.
	/// </summary>
	/// <param name="qualityValue">The graphics quality value to get display name for</param>
	/// <returns>Display name for the quality value</returns>
	private string GetGraphicsQualityDisplayName(string qualityValue)
	{
		// First check custom presets if available
		if (hasCustomPresets && graphicsPresets != null)
		{
			foreach (var preset in graphicsPresets)
			{
				if (preset.Value == qualityValue)
				{
					return preset.DisplayName;
				}
			}
		}
		
		// Fall back to default display names
		switch (qualityValue)
		{
			case "low":
				return "Low";
			case "medium":
				return "Medium";
			case "high":
				return "High";
			case "ultra":
				return "Ultra";
			default:
				return "Unknown";
		}
	}

	/// <summary>
	/// Applies the current volume setting to the audio system.
	/// </summary>
	private void ApplyVolumeSettings()
	{
		// TODO: Apply volume to actual audio system
		// Examples:
		// - Set master volume
		// - Update AudioListener volume
		// - Save volume to settings file
		// - Update audio mixer groups

		Log.Info($"Applied volume setting: {currentVolume:F2}");
	}

	/// <summary>
	/// Applies the current graphics quality setting to the graphics system.
	/// </summary>
	private void ApplyGraphicsQualitySettings()
	{
		// TODO: Apply graphics quality to actual graphics system
		// Examples:
		// - Set render quality level
		// - Update texture quality
		// - Adjust shadow quality
		// - Change anti-aliasing settings
		// - Update post-processing effects
		// - Save graphics settings to file

		Log.Info($"Applied graphics quality setting: {currentGraphicsQuality}");
	}

	/// <summary>
	/// Applies all current settings to the game systems.
	/// </summary>
	private void ApplyAllSettings()
	{
		ApplyVolumeSettings();
		ApplyGraphicsQualitySettings();
		
		// TODO: Apply other settings here
		// Examples:
		// - Control bindings
		// - Display settings
		// - Save settings to persistent storage

		Log.Info("All settings applied");
	}

}
