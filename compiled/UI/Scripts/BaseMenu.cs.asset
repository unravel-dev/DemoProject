using System;
using Unravel.Core;

/// <summary>
/// Base class for all menu controllers that eliminates code duplication.
/// Handles common UI setup, button event registration, and audio callbacks.
/// Child classes only need to implement their specific button actions.
/// </summary>
public abstract class BaseMenu : ScriptComponent
{
	// Cached UI wrapper objects
	protected UIDocument document;
	
	// Title element (common to all menus)
	protected UIElement titleElement;

	public override void OnStart()
	{
		// Get the UI document component
		var uiDoc = owner.GetComponent<UIDocumentComponent>();
		if (uiDoc == null)
		{
			Log.Error("No UIDocumentComponent found on entity");
			return;
		}

		// Get the document wrapper - this can be cached and reused
		document = uiDoc.GetDocument();
		if (document == null)
		{
			Log.Error("Failed to get document wrapper - document may not be loaded");
			return;
		}

		Log.Info($"Got document wrapper for {GetType().Name}: {document.Title}");

		// Cache element wrappers - these hold direct C++ pointers for fast access
		CacheUIElements();
		
		// Set up initial UI state
		SetupInitialUI();
		
		// Register event handlers using the wrapper objects
		RegisterEventHandlers();
	}

	/// <summary>
	/// Cache UI elements. Override in child classes to cache specific elements.
	/// </summary>
	protected virtual void CacheUIElements()
	{
		// Cache common title element - child classes should call base.CacheUIElements()
		titleElement = document.GetElementById(GetTitleElementId());
		
		// Log successful caching
		Log.Info($"Cached {CountValidElements()} UI elements successfully");
	}
	
	/// <summary>
	/// Count valid cached elements for logging. Override in child classes.
	/// </summary>
	protected virtual int CountValidElements()
	{
		int count = 0;
		if (titleElement?.IsValid() == true) count++;
		return count;
	}

	/// <summary>
	/// Set up initial UI state. Override in child classes for specific setup.
	/// </summary>
	protected virtual void SetupInitialUI()
	{
		// Set initial title text with current time
		if (titleElement != null)
		{
			var currentTime = DateTime.Now.ToString("HH:mm:ss");
			titleElement.SetAttribute("data-time", currentTime);
		}
		
		Log.Info($"{GetType().Name} UI setup completed");
	}

	/// <summary>
	/// Register event handlers. Override in child classes to register specific buttons.
	/// </summary>
	protected abstract void RegisterEventHandlers();

	/// <summary>
	/// Get the ID of the title element for this menu. Override in child classes.
	/// </summary>
	protected abstract string GetTitleElementId();

	public override void OnUpdate()
	{
		// Update time display in title
		if (titleElement != null)
		{
			var currentTime = DateTime.Now.ToString("HH:mm:ss");
			titleElement.SetAttribute("data-time", currentTime);
		}
	}

	// ========== COMMON BUTTON EVENT REGISTRATION ==========
	// Helper methods to reduce duplication in child classes

	/// <summary>
	/// Register all standard button events for a button element.
	/// </summary>
	/// <param name="button">The button element to register events for</param>
	/// <param name="buttonName">Name for logging purposes</param>
	/// <param name="onDown">Mouse down handler</param>
	/// <param name="onClick">Click handler</param>
	/// <param name="onHover">Mouse hover handler</param>
	/// <param name="onLeave">Mouse leave handler</param>
	/// <param name="onRelease">Mouse release handler</param>
	protected void RegisterButtonEvents(UIElement button, string buttonName,
		Action<UIPointerEvent> onDown,
		Action<UIPointerEvent> onClick,
		Action<UIPointerEvent> onHover,
		Action<UIPointerEvent> onLeave,
		Action<UIPointerEvent> onRelease)
	{
		if (button != null)
		{
			button.RegisterCallback<UIPointerEvent>("click", onClick);
			button.RegisterCallback<UIPointerEvent>("mousedown", onDown);
			button.RegisterCallback<UIPointerEvent>("mouseover", onHover);
			button.RegisterCallback<UIPointerEvent>("mouseout", onLeave);
			button.RegisterCallback<UIPointerEvent>("mouseup", onRelease);
			Log.Info($"{buttonName} button event handlers registered");
		}
		else
		{
			Log.Warning($"{buttonName} button not found or invalid");
		}
	}

	// ========== COMMON BUTTON EVENT HANDLERS ==========
	// Standard implementations that child classes can use

	/// <summary>
	/// Standard button down handler with audio feedback.
	/// </summary>
	protected void OnButtonDown(UIElement button, UIPointerEvent ev, string buttonName)
	{
		Log.Info($"{buttonName} button pressed down at position ({ev.x}, {ev.y}) with button {ev.button}");
		PlayButtonClickSound(button, ev);
		Log.Info($"{buttonName} button mouse down handled");
	}

	/// <summary>
	/// Standard button hover handler with audio feedback.
	/// </summary>
	protected void OnButtonHover(UIElement button, UIPointerEvent ev, string buttonName)
	{
		Log.Info($"{buttonName} button hovered at position ({ev.x}, {ev.y})");
		PlayButtonHoverSound(button, ev);
		Log.Info($"{buttonName} button hover handled");
	}

	/// <summary>
	/// Standard button leave handler.
	/// </summary>
	protected void OnButtonLeave(UIElement button, UIPointerEvent ev, string buttonName)
	{
		Log.Info($"{buttonName} button hover ended at position ({ev.x}, {ev.y})");
	}

	/// <summary>
	/// Standard button release handler.
	/// </summary>
	protected void OnButtonRelease(UIElement button, UIPointerEvent ev, string buttonName)
	{
		Log.Info($"{buttonName} button released at position ({ev.x}, {ev.y}) with button {ev.button}");
	}

	// ========== COMMON AUDIO CALLBACKS ==========
	// Shared audio functionality to avoid code duplication
	
	/// <summary>
	/// Plays a button click sound effect.
	/// Common callback used by all button click handlers.
	/// </summary>
	/// <param name="buttonElement">The button element that was clicked</param>
	/// <param name="ev">The pointer event containing click information</param>
	protected void PlayButtonClickSound(UIElement buttonElement, UIPointerEvent ev)
	{
		var clip = Assets.GetAsset<AudioClip>(AudioPaths.ButtonClick);
		AudioSourceComponent.PlayClipAtPoint(clip, new Vector3(0, 0, 0), 1f);
	}
	
	/// <summary>
	/// Plays a button hover sound effect.
	/// Common callback used by all button hover handlers.
	/// </summary>
	/// <param name="buttonElement">The button element that was hovered</param>
	/// <param name="ev">The pointer event containing hover information</param>
	protected void PlayButtonHoverSound(UIElement buttonElement, UIPointerEvent ev)
	{
		if (ev.targetElementId == buttonElement.ElementId)
		{
			var clip = Assets.GetAsset<AudioClip>(AudioPaths.ButtonHover);
			AudioSourceComponent.PlayClipAtPoint(clip, new Vector3(0, 0, 0), 1f);
		}
	}

	public override void OnDestroy()
	{
		// No need to manually clean up wrappers - they automatically become invalid
		// when the underlying C++ objects are destroyed
		// Event callbacks are also automatically cleaned up by the UIEventManager
		Log.Info($"{GetType().Name} controller destroyed");
	}
}

/// <summary>
/// Audio file path constants to avoid duplication and make changes easier.
/// </summary>
public static class AudioPaths
{
	public const string ButtonClick = "app:/data/UI/Sounds/button-click.mp3";
	public const string ButtonHover = "app:/data/UI/Sounds/button-hover.mp3";
}
