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

#include "CameraSimulator.h"
#include <iostream>
#include "message/input/Image.h"

const int TEX_WIDTH = 640;
const int TEX_HEIGHT = 480;
const double SCALE = 0.024;
const int NUM_NOISE_FRAMES = 1;

namespace module {
namespace simulation {

    uint8_t double_to_color(double d)
    {
        if (d < 0.0)
            d = 0.0;
        if (d > 1.0)
            d = 1.0;
        return (uint8_t)(d * 255.0);
    }

    CameraSimulator::CameraSimulator(std::unique_ptr<NUClear::Environment> environment)
    : Reactor(std::move(environment)) {
    
        is_initialised = false;

        // fill std::vector here??

        on<Always>().then([this] {

            // initialise on first call only

            if (!is_initialised)
            {
                is_initialised = true;
                initialise_ogre();
            }

            // update time info

            auto this_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_span = 
                    std::chrono::duration_cast<std::chrono::duration<double>>(this_time - last_time);
            last_time = this_time;

            // calculate flag positions, ball rotations etc.

            calculate_world(time_span);

            // render to window and texture

            ogre_root->renderOneFrame();
            window->swapBuffers();
            
            //render_target->update();
            //tex_to_yuyv();
            Ogre::WindowEventUtilities::messagePump();
            if (window->isClosed()) 
                abort();
        });
    }

    IGus CameraSimulator::new_igus()
    {
        IGus result;

        result.root_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        result.root_node->setScale(8.0, 8.0, 8.0);
        result.root_node->pitch(Ogre::Degree(90));
        result.root_node->roll(Ogre::Degree(180));

        Ogre::SceneNode* body_node = result.root_node->createChildSceneNode();
        Ogre::SceneNode* neck_node = body_node->createChildSceneNode();
        Ogre::SceneNode* shoulder_right_node = body_node->createChildSceneNode();
        Ogre::SceneNode* shoulder_left_node = body_node->createChildSceneNode();
        Ogre::SceneNode* bicep_right_node = shoulder_right_node->createChildSceneNode();
        Ogre::SceneNode* bicep_left_node = shoulder_left_node->createChildSceneNode();
        Ogre::SceneNode* forearm_right_node = bicep_right_node->createChildSceneNode();
        Ogre::SceneNode* forearm_left_node = bicep_left_node->createChildSceneNode();
        Ogre::SceneNode* hip_right_node = body_node->createChildSceneNode();
        Ogre::SceneNode* hip_left_node = body_node->createChildSceneNode();
        Ogre::SceneNode* thigh_right_node = hip_right_node->createChildSceneNode();
        Ogre::SceneNode* thigh_left_node = hip_left_node->createChildSceneNode();
        Ogre::SceneNode* lower_leg_right_node = thigh_right_node->createChildSceneNode();
        Ogre::SceneNode* lower_leg_left_node = thigh_left_node->createChildSceneNode();
        Ogre::SceneNode* foot_right_node = lower_leg_right_node->createChildSceneNode();
        Ogre::SceneNode* foot_left_node = lower_leg_left_node->createChildSceneNode();

        body_node->pitch(-Ogre::Degree(90));
        body_node->translate(-50.0 * SCALE,  19.1 * SCALE, -25.0 * SCALE);
        
        neck_node->translate(0.0, 10.0 * SCALE, 0.0);
        shoulder_right_node->translate(-3.2 * SCALE, 7.3 * SCALE, 0.0);
        shoulder_left_node->translate(3.2 * SCALE, 7.3 * SCALE, 0.0);
        bicep_right_node->translate(-1.8 * SCALE, -0.4 * SCALE, 0.0);
        bicep_left_node->translate(1.8 * SCALE, -0.4 * SCALE, 0.0);
        forearm_right_node->translate(-0.1 * SCALE, -6.5 * SCALE, 0.9 * SCALE);
        forearm_left_node->translate(-0.1 * SCALE, -6.5 * SCALE, 0.9 * SCALE);
        hip_right_node->translate(-2.3 * SCALE, 0.0 * SCALE, 0.0 * SCALE);
        hip_left_node->translate(2.3 * SCALE, 0.0 * SCALE, 0.0 * SCALE);
        thigh_right_node->translate(0.0 * SCALE, 2.4 * SCALE, 0.9 * SCALE);
        thigh_left_node->translate(0.0 * SCALE, 2.4 * SCALE, 0.9 * SCALE);
        lower_leg_right_node->translate(-0.2 * SCALE, -7.9 * SCALE, 0.1 * SCALE);
        lower_leg_left_node->translate(0.2 * SCALE, -7.9 * SCALE, 0.1 * SCALE);
        foot_right_node->translate(0.2 * SCALE, -9.7 * SCALE, -0.2 * SCALE);
        foot_left_node->translate(-0.2 * SCALE, -9.7 * SCALE, -0.2 * SCALE);
        
        Ogre::Entity* body = scene_mgr->createEntity("igus_body.mesh");
        body->setMaterialName("iguswhite");
        body->setCastShadows(false);
        body_node->attachObject(body);

        Ogre::Entity* head = scene_mgr->createEntity("igus_head.mesh");
        head->setMaterialName("iguswhite");
        head->setCastShadows(false);
        neck_node->attachObject(head);

        Ogre::Entity* eyemask = scene_mgr->createEntity("igus_eyemask.mesh");
        eyemask->setMaterialName("igusorange");
        eyemask->setCastShadows(false);
        neck_node->attachObject(eyemask);

        Ogre::Entity* shoulder_right = scene_mgr->createEntity("igus_shoulder_right.mesh");
        shoulder_right->setMaterialName("iguswhite");
        shoulder_right->setCastShadows(false);
        shoulder_right_node->attachObject(shoulder_right);

        Ogre::Entity* shoulder_left = scene_mgr->createEntity("igus_shoulder_left.mesh");
        shoulder_left->setMaterialName("iguswhite");
        shoulder_left->setCastShadows(false);
        shoulder_left_node->attachObject(shoulder_left);

        Ogre::Entity* bicep_right = scene_mgr->createEntity("igus_bicep_right.mesh");
        bicep_right->setMaterialName("iguswhite");
        bicep_right->setCastShadows(false);
        bicep_right_node->attachObject(bicep_right);

        Ogre::Entity* bicep_left = scene_mgr->createEntity("igus_bicep_left.mesh");
        bicep_left->setMaterialName("iguswhite");
        bicep_left->setCastShadows(false);
        bicep_left_node->attachObject(bicep_left);

        Ogre::Entity* forearm_right = scene_mgr->createEntity("igus_forearm_right.mesh");
        forearm_right->setMaterialName("iguswhite");
        forearm_right->setCastShadows(false);
        forearm_right_node->attachObject(forearm_right);

        Ogre::Entity* hand_right = scene_mgr->createEntity("igus_hand_right.mesh");
        hand_right->setMaterialName("igusorange");
        hand_right->setCastShadows(false);
        forearm_right_node->attachObject(hand_right);

        Ogre::Entity* forearm_left = scene_mgr->createEntity("igus_forearm_left.mesh");
        forearm_left->setMaterialName("iguswhite");
        forearm_left->setCastShadows(false);
        forearm_left_node->attachObject(forearm_left);

        Ogre::Entity* hand_left = scene_mgr->createEntity("igus_hand_left.mesh");
        hand_left->setMaterialName("igusorange");
        hand_left->setCastShadows(false);
        forearm_left_node->attachObject(hand_left);

        Ogre::Entity* hip_right = scene_mgr->createEntity("igus_hip_right.mesh");
        hip_right->setMaterialName("iguswhite");
        hip_right->setCastShadows(false);
        hip_right_node->attachObject(hip_right);

        Ogre::Entity* hip_left = scene_mgr->createEntity("Igushipleft", "igus_hip_left.mesh");
        hip_left->setMaterialName("iguswhite");
        hip_left->setCastShadows(false);
        hip_left_node->attachObject(hip_left);

        Ogre::Entity* thigh_right = scene_mgr->createEntity("Igusthighright", "igus_thigh_right.mesh");
        thigh_right->setMaterialName("iguswhite");
        thigh_right->setCastShadows(false);
        thigh_right_node->attachObject(thigh_right);

        Ogre::Entity* thigh_left = scene_mgr->createEntity("Igusthighleft", "igus_thigh_left.mesh");
        thigh_left->setMaterialName("iguswhite");
        thigh_left->setCastShadows(false);
        thigh_left_node->attachObject(thigh_left);

        Ogre::Entity* lower_leg_right = scene_mgr->createEntity("Iguslower_legright", "igus_lower_leg_right.mesh");
        lower_leg_right->setMaterialName("iguswhite");
        lower_leg_right->setCastShadows(false);
        lower_leg_right_node->attachObject(lower_leg_right);

        Ogre::Entity* lower_leg_left = scene_mgr->createEntity("Iguslower_legleft", "igus_lower_leg_left.mesh");
        lower_leg_left->setMaterialName("iguswhite");
        lower_leg_left->setCastShadows(false);
        lower_leg_left_node->attachObject(lower_leg_left);

        Ogre::Entity* foot_right = scene_mgr->createEntity("Igusfootright", "igus_foot_right.mesh");
        foot_right->setMaterialName("igusblack");
        foot_right->setCastShadows(false);
        foot_right_node->attachObject(foot_right);

        Ogre::Entity* arches_right = scene_mgr->createEntity("Igusarchesright", "igus_arches_right.mesh");
        arches_right->setMaterialName("iguswhite");
        arches_right->setCastShadows(false);
        foot_right_node->attachObject(arches_right);

        Ogre::Entity* foot_left = scene_mgr->createEntity("Igusfootleft", "igus_foot_left.mesh");
        foot_left->setMaterialName("igusblack");
        foot_left->setCastShadows(false);
        foot_left_node->attachObject(foot_left);

        Ogre::Entity* arches_left = scene_mgr->createEntity("Igusarchesleft", "igus_arches_left.mesh");
        arches_left->setMaterialName("iguswhite");
        arches_left->setCastShadows(false);
        foot_left_node->attachObject(arches_left);

        body_node->yaw(Ogre::Degree(90));

        return result;
    }

    void CameraSimulator::RenderNoise(Ogre::TexturePtr tex)
    {
        
        scene_mgr->getRootSceneNode()->attachObject(noise);
        
        params = noise->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
        params->setNamedConstant("seed", (Ogre::Real)((double) 10000000.0 * rand()/(double)RAND_MAX));      
        noise->setVisible(true);

        Ogre::RenderTexture* rt = tex->getBuffer()->getRenderTarget();
        
        rt->addViewport(camera);
        rt->getViewport(0)->setClearEveryFrame(true);
        rt->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Blue);
        rt->getViewport(0)->setOverlaysEnabled(false);
        rt->update();
        rt->removeViewport(0);
  
        noise->setVisible(false);
        scene_mgr->getRootSceneNode()->detachObject(noise);
        
    }

    void CameraSimulator::calculate_world(std::chrono::duration<double> time_span)
    {
        time_tally += time_span.count();
        //camera->yaw(Ogre::Radian(cos(time_tally * 2.0) / 30.0));
    }

    bool CameraSimulator::frameEnded(const Ogre::FrameEvent& evt)
    {
        flag0_anim->addTime(evt.timeSinceLastFrame);
        flag1_anim->addTime(evt.timeSinceLastFrame);
        flag2_anim->addTime(evt.timeSinceLastFrame);
        flag3_anim->addTime(evt.timeSinceLastFrame);

     //   params = noise->getMaterial()->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
       // params->setNamedConstant("seed", (Ogre::Real)(rand()/(double)RAND_MAX + 1.0));      
        //cur_noise_index = (cur_noise_index + 1) % NUM_NOISE_FRAMES;

    }

    void CameraSimulator::initialise_scene()
    {
       // Setup the basic scene (WARNING - wallpaper code)

        Ogre::SceneNode* base_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* base = scene_mgr->createEntity("stadiumstadionbase.mesh");
        base->setMaterialName("stadiumstadion_concrete");
        base->setCastShadows(true);
        base_node->attachObject(base);
        base_node->setScale(Ogre::Vector3(10.0f, 10.0f, 10.0f));
        base_node->yaw(Ogre::Degree(90));
        base_node->translate(0, 25.4f, -1.27f);

        Ogre::SceneNode* grass_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* grass = scene_mgr->createEntity("stadiumgrass.mesh");
        grass->setMaterialName("SoccerField");
        grass->setCastShadows(false);
        grass_node->attachObject(grass);
        grass_node->setScale(Ogre::Vector3(10.0f, 10.0f, 10.0f));
        grass_node->yaw(Ogre::Degree(90));
        grass_node->translate(0, 0.0333151f, -1.27f);

        Ogre::SceneNode* flag_node0 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::SceneNode* flag_pole_node0 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* flag0 = scene_mgr->createEntity("stadiumFlag_Cloth.mesh");
        flag0->setMaterialName("soccerballMaterial__26");
        Ogre::Entity* flag_pole0 = scene_mgr->createEntity("stadiumFlag_Pole.mesh");
        flag_pole0->setMaterialName("soccerballMaterial__25");
        Ogre::Entity* flag_ring0 = scene_mgr->createEntity("stadiumPole_Cloth.mesh");
        flag_ring0->setMaterialName("stadiumMaterial__28");
        flag_node0->attachObject(flag0);
        flag_pole_node0->attachObject(flag_pole0);
        flag_pole_node0->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_pole_node0->translate(30.28, 0.0, -22.76);
        flag_pole_node0->attachObject(flag_ring0);
        flag_node0->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_node0->pitch(Ogre::Degree(90));
        flag_node0->translate(30.28, 3.2, -22.76);
        flag_node0->roll(Ogre::Degree(69 - 90));
        flag0_anim = flag0->getAnimationState("default_morph");
        flag0_anim->setEnabled(true);
        flag0_anim->setLoop(true);


        Ogre::SceneNode* flag_node1 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::SceneNode* flag_pole_node1 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* flag1 = scene_mgr->createEntity("stadiumFlag_Cloth.mesh");
        flag1->setMaterialName("soccerballMaterial__26");
        Ogre::Entity* flag_pole1 = scene_mgr->createEntity("stadiumFlag_Pole.mesh");
        flag_pole1->setMaterialName("soccerballMaterial__25");
        Ogre::Entity* flag_ring1 = scene_mgr->createEntity("stadiumPole_Cloth.mesh");
        flag_ring1->setMaterialName("stadiumMaterial__28");
        flag_node1->attachObject(flag1);
        flag_pole_node1->attachObject(flag_pole1);
        flag_pole_node1->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_pole_node1->translate(-30.28, 0.0, -22.76);
        flag_pole_node1->attachObject(flag_ring1);
        flag_node1->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_node1->pitch(Ogre::Degree(90));
        flag_node1->translate(-30.28, 3.2, -22.76);
        flag_node1->roll(Ogre::Degree(69 - 90));
        flag1_anim = flag1->getAnimationState("default_morph");
        flag1_anim->setEnabled(true);
        flag1_anim->setLoop(true);


        Ogre::SceneNode* flag_node2 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::SceneNode* flag_pole_node2 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* flag2 = scene_mgr->createEntity("stadiumFlag_Cloth.mesh");
        flag2->setMaterialName("soccerballMaterial__26");
        Ogre::Entity* flag_pole2 = scene_mgr->createEntity("stadiumFlag_Pole.mesh");
        flag_pole2->setMaterialName("soccerballMaterial__25");
        Ogre::Entity* flag_ring2 = scene_mgr->createEntity("stadiumPole_Cloth.mesh");
        flag_ring2->setMaterialName("stadiumMaterial__28");
        flag_node2->attachObject(flag2);
        flag_pole_node2->attachObject(flag_pole2);
        flag_pole_node2->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_pole_node2->translate(-30.28, 0.0, 20.26);
        flag_pole_node2->attachObject(flag_ring2);
        flag_node2->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_node2->pitch(Ogre::Degree(90));
        flag_node2->translate(-30.28, 3.2, 20.26);
        flag_node2->roll(Ogre::Degree(69 - 90));
        flag2_anim = flag2->getAnimationState("default_morph");
        flag2_anim->setEnabled(true);
        flag2_anim->setLoop(true);

        Ogre::SceneNode* flag_node3 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::SceneNode* flag_pole_node3 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* flag3 = scene_mgr->createEntity("stadiumFlag_Cloth.mesh");
        flag3->setMaterialName("soccerballMaterial__26");
        Ogre::Entity* flag_pole3 = scene_mgr->createEntity("stadiumFlag_Pole.mesh");
        flag_pole3->setMaterialName("soccerballMaterial__25");
        Ogre::Entity* flag_ring3 = scene_mgr->createEntity("stadiumPole_Cloth.mesh");
        flag_ring3->setMaterialName("stadiumMaterial__28");
        flag_node3->attachObject(flag3);
        flag_pole_node3->attachObject(flag_pole3);
        flag_pole_node3->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_pole_node3->translate(30.28, 0.0, 20.26);
        flag_pole_node3->attachObject(flag_ring3);
        flag_node3->setScale(Ogre::Vector3(1.5f, 1.5f, 1.5f));
        flag_node3->pitch(Ogre::Degree(90));
        flag_node3->translate(30.28, 3.2, 20.26);
        flag_node3->roll(Ogre::Degree(69 - 90));
        flag3_anim = flag3->getAnimationState("default_morph");
        flag3_anim->setEnabled(true);
        flag3_anim->setLoop(true);
 
        Ogre::SceneNode* chairs_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* chairs = scene_mgr->createEntity("stadiumchairs.mesh");
        chairs->setMaterialName("stadiumchairs");
        chairs->setCastShadows(false);
        chairs_node->attachObject(chairs);
        chairs_node->setScale(Ogre::Vector3(10.1f, 10.0f, 10.0f));
        chairs_node->yaw(Ogre::Degree(76.5));
        chairs_node->translate(-48.8449, 12.6779, 47.0319);

        Ogre::SceneNode* lines_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* lines = scene_mgr->createEntity("stadiumsoccerlines.mesh");
        lines->setMaterialName("stadiumwhite");
        lines->setCastShadows(false);
        lines_node->attachObject(lines);
        lines_node->setScale(Ogre::Vector3(10.0f, 10.0f, 10.0f));
        lines_node->yaw(Ogre::Degree(90));
        lines_node->translate(0, 0.0357773, -1.27);

        ball_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* ball = scene_mgr->createEntity("soccerballFootball.mesh");
        ball->setMaterialName("SoccerBall");
        ball_node->attachObject(ball);
        ball_node->setScale(Ogre::Vector3(0.4, 0.4, 0.4));
        ball_node->translate(22.0, 0.8, 0.0);
        
        Ogre::SceneNode* goal_a_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* goal_a = scene_mgr->createEntity("stadiumgoalnet01.mesh");
        goal_a->setMaterialName("stadiumnet");
        goal_a->setCastShadows(true);
        goal_a_node->attachObject(goal_a);
        Ogre::Entity* goal_posts = scene_mgr->createEntity("stadiumgoalframe01.mesh");
        Ogre::SceneNode* posts_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        goal_posts->setMaterialName("stadiumwhite");
        goal_posts->setCastShadows(true);
        posts_node->attachObject(goal_posts);
        posts_node->setScale(Ogre::Vector3(10.0, 10.0, 10.0));
        posts_node->translate(30.5, 0.12, -7.9);
        posts_node->roll(Ogre::Degree(90));
        goal_a_node->setScale(Ogre::Vector3(10.0, 10.0, 10.0));
        goal_a_node->yaw(Ogre::Degree(90));
        goal_a_node->translate(32.75, 0.12, -1.18);
        
        Ogre::SceneNode* goal_b_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* goal_b = scene_mgr->createEntity("GoalB", "stadiumgoalnet01.mesh");
        goal_b->setMaterialName("stadiumnet");
        goal_b->setCastShadows(true);
        goal_b_node->attachObject(goal_b);
        Ogre::Entity* goal_postsb = scene_mgr->createEntity("stadiumgoalframe01.mesh");
        Ogre::SceneNode* posts_nodeb = scene_mgr->getRootSceneNode()->createChildSceneNode();
        goal_postsb->setMaterialName("stadiumwhite");
        goal_postsb->setCastShadows(true);
        posts_nodeb->attachObject(goal_postsb);
        posts_nodeb->setScale(Ogre::Vector3(10.0, 10.0, 10.0));
        posts_nodeb->translate(-30.5, 0.12, -7.9);
        posts_nodeb->roll(Ogre::Degree(90));
        goal_b_node->setScale(Ogre::Vector3(10.0, 10.0, 10.0));
        goal_b_node->yaw(-Ogre::Degree(90));
        goal_b_node->translate(-32.75, 0.12, -1.18);
        
        Ogre::Entity* logo = scene_mgr->createEntity(Ogre::SceneManager::PT_PLANE);
        logo->setMaterialName("Logo");
        logo->setCastShadows(false);
        Ogre::SceneNode* logo_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        logo_node->setScale(Ogre::Vector3(0.0300, 0.008, 1.0f));
        logo_node->yaw(Ogre::Degree(180));
        logo_node->translate(16.0f, 1.0f, 21.9f);
        logo_node->attachObject(logo);

        Ogre::Entity* logo2 = scene_mgr->createEntity(Ogre::SceneManager::PT_PLANE);
        logo2->setMaterialName("Logo");
        logo2->setCastShadows(false);
        Ogre::SceneNode* logo_node2 = scene_mgr->getRootSceneNode()->createChildSceneNode();
        logo_node2->setScale(Ogre::Vector3(0.0300, 0.0080, 1.0f));
        logo_node2->yaw(Ogre::Degree(180));
        logo_node2->translate(-16.0f, 1.0f, 21.9f);
        logo_node2->attachObject(logo2);

        igus = new_igus();

        noise_tex0 = Ogre::TextureManager::getSingleton().createManual
        (
                "NoiseTex0", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
                640, 480, 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET
        );

         noise = new Ogre::Rectangle2D(true);
         noise->setCorners(-1.0, 1.0, 1.0, -1.0);
         noise->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
         noise->setMaterial("NoiseGenerator");
         noise->setCastShadows(false);
         RenderNoise(noise_tex0);

        // for (int i=0; i<NUM_NOISE_FRAMES; i++)
        // {
        //     Ogre::String name = Ogre::String("NoiseTex") + (char)(48+i);
        //     noise_tex.push_back(Ogre::TextureManager::getSingleton().createManual
        //     (
        //         name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        //         640, 480, 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET

        //     ));
        //     //RenderNoise(noise_tex[i]);
        // }*/
            
    }

    void CameraSimulator::initialise_ogre()
    {
        // Set up OGRE root!!

        ogre_root = new Ogre::Root("plugins.cfg");
 
        // Go through all sections & settings in the file

        Ogre::ConfigFile cf;
        cf.load("resources.cfg");

        Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

        Ogre::String sec_name, type_name, arch_name;
        while (seci.hasMoreElements())
        {
            sec_name = seci.peekNextKey();
            Ogre::ConfigFile::SettingsMultiMap* settings = seci.getNext();
            Ogre::ConfigFile::SettingsMultiMap::iterator i;
            for (i = settings->begin(); i != settings->end(); ++i)
            {
                type_name = i->first;
                arch_name = i->second;
                Ogre::ResourceGroupManager::getSingleton().addResourceLocation(arch_name, type_name, sec_name);

            }
        }

        if (ogre_root->restoreConfig() == 0)
        {
            std::cout << "Failure to load ogre.cfg\n";
            return;
        }
       
        // Setup the hidden window

        window = ogre_root->initialise(true, "NUSimulator");
       // window->setHidden(true);
        //Ogre::WindowEventUtilities::messagePump(); // force hiding the window

        Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

        Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
        Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_ANISOTROPIC);
        Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(8);
 
        // Setup lights/cameras
 
        scene_mgr = ogre_root->createSceneManager(Ogre::ST_GENERIC);

        scene_mgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
        scene_mgr->setAmbientLight(Ogre::ColourValue(0.5f, 0.5f, 0.5f));

        camera = scene_mgr->createCamera("PlayerCam");

        camera->setNearClipDistance(5);
        camera->setAspectRatio((double)TEX_WIDTH/(double)TEX_HEIGHT);

        scene_mgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));

        scene_mgr->setSkyDome(true, "Examples/CloudySky", 5, 8);

        Ogre::Light* light0 = scene_mgr->createLight();
        light0->setPosition(20, 80, 50);
        light0->setDiffuseColour(1.0, 1.0, 1.0);
 
        float pitch = -0.18f;
        float yaw = 1.8f;

        float x = -20.0f;
        float y = 8.0f;
        float z = -5.0f;
        
        camera->setPosition(x, y, z);
        camera->setDirection(sin(yaw) * cos(pitch), sin(pitch), -cos(yaw) * cos(pitch));   
 
        Ogre::Viewport* vp = window->addViewport(camera);
        vp->setBackgroundColour(Ogre::ColourValue(0,0,0));
 
        ogre_root->addFrameListener(this);


        initialise_scene();

        // setup render to texture

        rtt_tex = Ogre::TextureManager::getSingleton().createManual("RttTex", 
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,  Ogre::TEX_TYPE_2D, 
                TEX_WIDTH, TEX_HEIGHT, 1,  Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

//        render_target = rtt_tex->getBuffer()->getRenderTarget();
 
 //       render_target->addViewport(camera);
 //       render_target->getViewport(0)->setClearEveryFrame(true);
 //       render_target->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
 //       render_target->getViewport(0)->setOverlaysEnabled(false);
        window->addListener(this);

        image_yuyv.resize(TEX_WIDTH * TEX_HEIGHT * 2);
        last_time = std::chrono::steady_clock::now();
        time_tally = 0;

        cur_noise_index = 0;

        noise_node = scene_mgr->getRootSceneNode()->createChildSceneNode();
        screen_noise = new Ogre::Rectangle2D(true);
        screen_noise->setCorners(-0.0, 0.0, 1.0, -1.0);
        screen_noise->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);
        screen_noise->setCastShadows(false);
        screen_noise->setMaterial("NoiseMaterial");
        screen_noise->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY);
        screen_noise->setVisible(true);
        noise_node->attachObject(screen_noise);
    }


    void CameraSimulator::preRenderTargetUpdate(const Ogre::RenderTargetEvent& rte)
    {
        RenderNoise(noise_tex0);
        screen_noise->setVisible(true);

      //  screen_mb->setVisible(true);
    }

    void CameraSimulator::postRenderTargetUpdate(const Ogre::RenderTargetEvent& rte)
    {
    //    screen_mb->setVisible(false);
        screen_noise->setVisible(false);
    }

    void CameraSimulator::tex_to_yuyv()
    {
        Ogre::HardwarePixelBufferSharedPtr ptr = rtt_tex->getBuffer(0,0);
        if (ptr->getFormat() != Ogre::PixelFormat::PF_X8R8G8B8)
        {
            std::cout << "BAD IMAGE FORMAT\n";
            abort();
        }
        ptr->lock(Ogre::HardwareBuffer::HBL_READ_ONLY);
        const Ogre::PixelBox& pixel_box = ptr->getCurrentLock();
        unsigned char* bitmap = static_cast<unsigned char*>(pixel_box.data);

        for (int i=0; i<TEX_WIDTH * TEX_HEIGHT * 4; i += 8)
        {
            double blue1 = (double) bitmap[i];
            double green1 = (double) bitmap[i + 1];
            double red1 = (double) bitmap[i + 2];

            double blue2 = (double) bitmap[i + 4];
            double green2 = (double)bitmap[i + 5];
            double red2 =  (double)bitmap[i + 6];

            double y1 = 0.299 * red1 + 0.587 * green1 + 0.114 * blue1;
            double y2 = 0.299 * red2 + 0.587 * green2 + 0.114 * blue2;

            double u1 = (0.492 * (blue1 - y1) + 0.436) / 2.0;
            double u2 = (0.492 * (blue2 - y2) + 0.436) / 2.0;
            double v1 = (0.877 * (red1 - y1) + 0.615) / 2.0;
            double v2 = (0.877 * (red2 - y2) + 0.615) / 2.0;
            
            int j = i / 2;
            image_yuyv[j] = double_to_color(y1); 
            image_yuyv[j + 1] = double_to_color((u1 + u2) / 2.0);
            image_yuyv[j + 2] = double_to_color(y2);
            image_yuyv[j + 3] = double_to_color((v1 + v2) / 2.0);
        }
        ptr->unlock();
    }
}
}

