using System;
using System.Runtime.CompilerServices;
using Unravel.Core;

[ScriptSourceFile]
class JumpPlatform : ScriptComponent
{
	[Range(5, 15)]
	public int JumpForce = 12;
	/// <summary>
	/// OnCreate is called when the script is first loaded, or when an object it is attached to is instantiated
	/// It only gets called once on each script, and only after other objects are initialised.
	/// This means that it is safe to create references to other game objects and components in OnCreate.
	/// </summary>
	public override void OnCreate()
	{
	}

	/// <summary>
	/// Start is called once, before any Update methods and after OnCreate.
	/// It works in much the same way as OnCreate, with a few key differences.
	/// Unlike OnCreate, Start will not be called if the script is disabled.
	/// </summary>
	public override void OnStart()
	{
	}
	public override void OnSensorEnter(Collision c)
	{
		//if (e.name != "Player")
		//	return;

		var physics = c.entity.GetComponent<PhysicsComponent>();
		if (physics != null)
		{
			physics.ApplyForce(Vector3.up * JumpForce, ForceMode.Impulse);
			// physics.ApplyForce(e.transform.forward * 10.0f, ForceMode.Impulse);

		}
		
	

    }
}