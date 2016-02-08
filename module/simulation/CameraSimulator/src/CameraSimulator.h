/*
 * This file is part of NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2016 NUbots <nubots@nubots.net>
 */

#ifndef MODULE_SIMULATOR_CAMERASIMULATOR_H
#define MODULE_SIMULATOR_CAMERASIMULATOR_H

#include <nuclear>
#include <vector>
#include <chrono>

#include <Overlay/OgreOverlay.h>
#include <OgreEntity.h>
#include <OgreCamera.h>
#include <OgreLogManager.h>
#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreTextureManager.h>
#include <OgreWindowEventUtilities.h>
#include <OgreCompositorInstance.h>
#include <OgreCompositorLogic.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRenderTargetListener.h>

namespace module {
namespace simulation {

	class WorldState {

	public:

		Ogre::Vector3 camera_pos;
		Ogre::Real camera_pitch;
		Ogre::Real camera_yaw;

		Ogre::Vector3 ball_pos;
		Ogre::Vector3 last_ball_pos;
	};

	class IGus
	{

	public:

		Ogre::SceneNode* root_node;
	};

    class CameraSimulator : public NUClear::Reactor, Ogre::FrameListener, Ogre::RenderTargetListener {

    	Ogre::Root* ogre_root;
    	Ogre::Camera* camera;
    	Ogre::SceneManager* scene_mgr;
    	Ogre::RenderWindow* window;
    	Ogre::RenderTexture* render_target;
		Ogre::TexturePtr rtt_tex;

		bool is_initialised;

		std::chrono::steady_clock::time_point last_time;
		double time_tally;

		IGus igus;

		Ogre::AnimationState* flag0_anim;		
		Ogre::AnimationState* flag1_anim;
		Ogre::AnimationState* flag2_anim;
		Ogre::AnimationState* flag3_anim;

		Ogre::SceneNode* ball_node;
    	Ogre::GpuProgramParametersSharedPtr params;

		Ogre::SceneNode* noise_node;
		std::vector<Ogre::TexturePtr> noise_tex;
		Ogre::Rectangle2D* noise;
		Ogre::Rectangle2D* screen_noise;
    	int cur_noise_index;
		Ogre::TexturePtr noise_tex0;
		std::vector<uint8_t> image_yuyv;

   	private:

   		void initialise_scene();
   		void initialise_ogre();
   		void calculate_world(std::chrono::duration<double> time_span);
   		void tex_to_yuyv();
   		IGus new_igus();


    public:
        /// @brief Called by the powerplant to build and setup the CameraSimulator reactor.
   		virtual bool frameEnded(const Ogre::FrameEvent& evt);
   		virtual void preRenderTargetUpdate(const Ogre::RenderTargetEvent& rte);
   		virtual void postRenderTargetUpdate(const Ogre::RenderTargetEvent& rte);
   		void RenderNoise(Ogre::TexturePtr tex);
        explicit CameraSimulator(std::unique_ptr<NUClear::Environment> environment);
    };

}
}

#endif  // MODULE_SIMULATOR_CAMERASIMULATOR_H
