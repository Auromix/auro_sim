# auro_sim

Isaac sim wrapper for auromix.

## scripts

- config_isaac_sim_environment.sh

  Run script to config isaac sim environment in bashrc.

```bash
script_name="config_isaac_sim_environment.sh" && directory_name="/tmp" && download_url="https://raw.githubusercontent.com/Auromix/auro_sim/main/scripts" && full_script_path="${directory_name}/${script_name}" && wget -O $full_script_path $download_url/$script_name && sudo chmod +x $full_script_path && clear && bash $full_script_path && rm -rf $full_script_path
```

- copy_isaac_sim_local_assets.sh

```bash
script_name="copy_isaac_sim_local_assets.sh" && directory_name="/tmp" && download_url="https://raw.githubusercontent.com/Auromix/auro_sim/main/scripts" && full_script_path="${directory_name}/${script_name}" && wget -O $full_script_path $download_url/$script_name && sudo chmod +x $full_script_path && clear && bash $full_script_path && rm -rf $full_script_path
```

- setup_isaac_sim_ide_environment.sh

Run the script in the top-level directory of your workspace to set up the Isaac Sim environment for VS Code, enabling the IDE to recognize the Isaac Sim package.
```bash
script_name="setup_isaac_sim_ide_environment.sh" && directory_name="/tmp" && download_url="https://raw.githubusercontent.com/Auromix/auro_sim/main/scripts" && full_script_path="${directory_name}/${script_name}" && wget -O $full_script_path $download_url/$script_name && sudo chmod +x $full_script_path && bash $full_script_path && rm -rf $full_script_path
```
