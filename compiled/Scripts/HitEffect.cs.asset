using System;
using System.Runtime.CompilerServices;
using Unravel.Core;
using System.Collections.Generic;
using System.Linq.Expressions;

[ScriptSourceFile]
class HitEffects : ScriptComponent
{
	public int collisions_ = 0;
	public int total_hits_ = 0;

	private ModelComponent model_ = null;
	public override void OnStart()
	{
		model_ = owner.GetComponent<ModelComponent>();
		
    }

	public override void OnSensorEnter(Collision c)
	{
		if (c.entity.name == "Player")
			Log.Info($"OnSensorEnter {c.entity}");
		//SetColor(Color.red);
		SetColor(new Color(1.0f, 0.0f, 0.0f, 0.55f));
		collisions_++;
		total_hits_++;

		var text = owner.GetComponentInChildren<TextComponent>();
		if (text != null)
		{
			text.text = $"{total_hits_}";
		}
		
	

    }
    public override void OnSensorExit(Collision c)
    {
		if(c.entity.name == "Player")
			Log.Info($"OnSensorExit {c.entity}");

		collisions_--;
		if(collisions_ == 0)
		{
			RestoreColor();
		}

    }

	public override void OnCollisionEnter(Collision collision)
	{
		if (collision.entity.name == "Player")
			Log.Info($"OnCollisionEnter {collision}");
		SetColor(Random.color);
		collisions_++;
		total_hits_++;

		var text = owner.GetComponentInChildren<TextComponent>();
		if (text != null)
		{
			text.text = $"{total_hits_}";
		}
    }
  
    public override void OnCollisionExit(Collision collision)
    {
		if(collision.entity.name == "Player")
			Log.Info($"OnCollisionExit {collision}");

		collisions_--;
		if(collisions_ == 0)
		{
			RestoreColor();
		}
		
    }

	private void SetColor(Color c)
	{
		if(model_ != null)
		{
			model_.SetColor(c);
		}
	}

	private void RestoreColor()
	{
		if(model_ != null)
		{
			model_.ResetMaterials();
		}
	}


}