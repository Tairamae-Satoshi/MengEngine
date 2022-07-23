# README

####Introduction

This is a skeleton animation engine for studying character animation by implementing them.



#### Features:

* Basic Animation Feature
  * Skeleton Animation
  * Animation sampling
  * Animation blending
    - linear blending
    - Inertialization

  * Linear skinning
* Advanced Animation Features
  * Blendspace
    * Motion analysis (used to align the key time)
    * Gradient band interpolation
  * Motion Matching
    * Spring Controller
    * Trajectory matching
    * Pose matching(Only position for now)
  * IK Rigging
    * Two bone IK
    * Aim IK



####Examples

* Animation Blending

  ![AnimationBlend](/Contents/Gif/AnimationBlending.gif)

* Two Bone IK and Look at IK

  ![AnimationBlend](/Contents/Gif/TwoBoneIK_LookIK.gif)

* Blend Space

  ![AnimationBlend](/Contents/Gif/BlendSpace.gif)

* Motion Matching

  ![AnimationBlend](/Contents/Gif/MotionMatching.gif)

* IK Rigging

  ![AnimationBlend](/Contents/Gif/IKRigging.gif)

  

#### References:

JOHANSEN, R. S. 2009. Automated Semi-Procedural Animation for Character Locomotion. Master’s thesis, Aarhus University. 

Park, S. I., Shin, H. J., and Shin, S. Y. (2002). On-line locomotion generation based on motion blending. In SCA ’02: Proceedings of the 2002 ACMSIGGRAPH/Eurographics symposium on Computer animation, pages 105–111, New York, NY, USA. ACM.

Simon Clavet Motion Matching and The Road to Next-Gen Animation 

Daniel Holden, Oussama Kanoun, Maksym Perepichka, and Tiberiu Popa. 2020. Learned motion matching. ACM Trans. Graph. 39, 4, Article 53 (August 2020), 13 pages. https://doi.org/10.1145/3386569.3392440

David Bollo. Inertialization: High-Performance Animation Transitions in 'Gears of War', GDC 2018. 

Alexander Bereznyak IK Rig: Procedural Pose Animation