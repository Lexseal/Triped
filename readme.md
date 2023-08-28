# Tripedal Robot

This is a repo containing hardware, electrical and software components of a tripedal walker robot.

## Motivation

Ever since the early 2000s, people have been developing quadrupeds and bipeds such as robot dogs and humanoids. However, there is less effort directed towards tripedal robots. Although they lack the stability of quadrupedal robots and also may be more mechanically complex than bipedal robots, tripeds are interesting in two regards:
1. They are more techincally challenging than other widely studied robots. They can represent a robot dog with one missing leg.
2. They are more stable than bipeds while also being a bit cheaper than quadrupeds since they have fewer legs that need to be actuated.

## High-level Goals (WIP)

- [x] Take inspration from earlier work and minic similar designs
- [x] Purchase equipments (printers, servos, etc.)
- [ ] Develop feedforward walking algorithm through trial and error
- [ ] Plan sensors and compute
- [ ] Initially have everything on breadboards but slowly transition to PCB
- [ ] Interate mechanically while developing feedback walking algorithms that accomendate a wider range of movements such as jumping, turning, walking on hills, and walking on rough/slippery terrain
- [ ] Set up gymnasium+pybullet environment for reinforcement learning

## Simulation Software

After some back and forth and reading [this article](https://arxiv.org/pdf/2103.04616.pdf), I have decided to go with [Pybullet](https://github.com/bulletphysics/bullet3). Although that is the slowest environment to run, most of the RL training time is spent on actual training rather than the simulation itself. Thus, Pybullet's ease of use, its native python interface, and big community is a huge plus.

## Mesh Simplification

Before any simplification and using the original stl as collision body, the simulation ran 211fps.

After removing all the collision bodies other than the feet, the simulation ran 222fps.

After reducing the complexity of the feet, the simulation ran 208fps???

Seems to be within run to run variation. I guess the simplification is not worth it.
