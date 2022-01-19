## Octomap Merging Package
This is a simplified version of the Marble Mapping package (currently forked) I created to test out a map alignment procedure to align incoming neighbor maps to the self robot's map. I should note that the package currently only works with diff maps but once it's integrated into Marble Mapping, it should be able to process point clouds.
<br/>

### Before alignment procedure:
![Alt Text](https://media.giphy.com/media/alQeTHjIgPUTipX1tg/giphy.gif)
<br/>
The misalignment is most noticeable when looking at the neon green map section. 

### After alignment procedure:
![Alt Text](https://media.giphy.com/media/rjeL0F9ZdAWColVGqi/giphy.gif)
<br/>
Once it's corrected, it's actual useful for any multi-robot stuff you want to do.

To run the package, configure the statistical outlier filter and ICP settings through the icp_param_config.yaml file and change the name of the diff map input topics in the octomap_merging.launch file. 
<br/><br/>
I'm still working on implementing these changes into the Marble Mapping package. Grad school is hard but I'll get back to this eventually. 
