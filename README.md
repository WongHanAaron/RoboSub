 # The Embedded Systems Team Repository

The Embedded System repository is composed of three main components:
* 1. **Exodus:** the official firmware that is flashed onto the TI and used for competition
* 2. **PrototypePlayground:** a playground project for users to test various functions and new development
* 3. **Libraries:** the remaining C++ files that serve as libraries for the Exodus project
* 3. (a) **System:** contains the implementation of various device drivers
* 3. (b) **Sensors:** contains the implementation for various sensors
* 3. (c) **Utilities:** contains the implementation for various commonly used on-board functionalities

## Wiki
This readme serves as a quick reference of the directories available. For detailed documentation, resources, and quick-start tutorials, refer to the GitLab wiki found at https://gitlab.com/GU-RoboSub/Embedded-System/wikis/home


## Process for using GitLab
1. Fork the project (i.e. get your own personal copy of the code)
2. Create a new branch when creating a new driver (known as a feature branch)
3. Checkout into your feature branch and prototype your new driver in the **PrototypePlayground** project
4. Integrate your new driver into the **Exodus** exodus project
5. Commit your changes to this feature branch
6. Push your changes to GitLab
7. Create a merge request
