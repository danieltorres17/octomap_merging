## The Octomap Merging Package - For Online Multi-Robot 3D Octomap Merging
This is a simplified version of the Marble Mapping package (which currently I have forked) I created to test out a map alignment procedure which aligns incoming neighbor maps to the self robot's map via Generalized-ICP. This proposed change fixes a few of the common sources of misalignment I saw while using Marble Mapping such as initial world reference frame error which propagated as the robot's explored deeper into the environment, odometry drift, sensor noise, etc. I should note that the package currently only works with diff maps but once it's integrated into Marble Mapping, it should be able to process point clouds as inputs. I'm still working on implementing these changes into the Marble Mapping package. Grad school is time-consuming but I'll get back to this eventually.
<br/>

### Before alignment procedure:
![Alt Text](https://media.giphy.com/media/alQeTHjIgPUTipX1tg/giphy.gif)
<br/><br/>
The misalignment is most noticeable when looking at the neon green map section towards the left. 

### After alignment procedure:
![Alt Text](https://media.giphy.com/media/rjeL0F9ZdAWColVGqi/giphy.gif)
<br/><br/>
Once it's corrected, it's actual useful for any multi-robot stuff you want to do.

To run the package, configure the statistical outlier filter and ICP settings through the icp_param_config.yaml file and change the name of the diff map input topics in the octomap_merging.launch file. I'll write more detailed instructions soon!
