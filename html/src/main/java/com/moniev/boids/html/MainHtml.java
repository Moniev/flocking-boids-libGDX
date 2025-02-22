package com.moniev.boids.html;

import com.moniev.boids.core.Main;

import com.badlogic.gdx.ApplicationListener;
import com.badlogic.gdx.backends.gwt.GwtApplication;
import com.badlogic.gdx.backends.gwt.GwtApplicationConfiguration;

public class MainHtml extends GwtApplication {
	@Override
	public ApplicationListener getApplicationListener () {
		return new Main();
	}
	
	@Override
	public GwtApplicationConfiguration getConfig () {
		return new GwtApplicationConfiguration(480, 320);
	}
}
