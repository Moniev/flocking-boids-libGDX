package com.moniev.boids.java;

import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.moniev.boids.core.Main;

public class MainDesktop {
	public static void main (String[] args) {
		Lwjgl3ApplicationConfiguration config = new Lwjgl3ApplicationConfiguration();
    config.setTitle("Boids Flocking Simulation");
    config.setWindowedMode(1440, 1440);
    config.useVsync(true);
    new Lwjgl3Application(new Main(), config);
	}
}
