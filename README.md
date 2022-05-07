# rasterizer
This program implements a simple model of a modern graphics pipeline. It takes geometry data as an input and produces an image. The image is then output to the console as an array of ASCII characters. The geometry data consists of an array of vertices and an array of indices that represent triangles. The vertices have different attributes such as their position, normals and shade. The program generates a spherical mesh as the input geometry. The vertices are transformed in the vertex_shader function, then the triangles are rasterized and the resulting fragments are shaded in the fragment_shader function.

The rendered image:
```
                                       ****##################                                       
                               ********##############################                               
                           **********####################################                           
                       =*************########################################                       
                    ==***************###########################################                    
                 ===******************############################################*                 
               =====********************###########################################**               
             +=====***********************##########################################***             
            ========***********************#########%%%%%%%##########################***            
          +=========***********************######%%@@@@@@@@%%%######################******          
         ++==========**********************####%%@@@@@@@@@@@@@%%##################*********         
        +++===========**********************###%%@@@@@@@@@@@@@@%%#############**************        
       ++++============**********************###%%@@@@@@@@@@@@%%######***********************       
      :++++==============*********************###%%%@@@@@@@@@%%#####*************************=      
      ++++++===============*********************#####%%%%%%%#####****************************=      
     :+++++++=================***********************########********************************==     
     ::+++++++===================***********************************************************===     
     ::++++++++======================******************************************************====     
     :::+++++++++========================***********************************************=======     
     ::::++++++++++===========================***************************************==========     
     -::::+++++++++++=================================**************************==============+     
      -::::+++++++++++++=====================================================================+      
      -::::::++++++++++++++=================================================================++      
       --::::::+++++++++++++++============================================================+++       
        --:::::::+++++++++++++++++=====================================================+++++        
         ---:::::::+++++++++++++++++++++===========================================++++++++         
          .---::::::::+++++++++++++++++++++++++===============================+++++++++++:          
            ----::::::::::+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++:            
             .----::::::::::::++++++++++++++++++++++++++++++++++++++++++++++++++++++:::             
               .-----::::::::::::::+++++++++++++++++++++++++++++++++++++++++++++:::::               
                 ..------::::::::::::::::++++++++++++++++++++++++++++++++++:::::::-                 
                    ..-------:::::::::::::::::::::::::+++++++++::::::::::::::::-                    
                       ...--------:::::::::::::::::::::::::::::::::::::::::--                       
                           ...------------:::::::::::::::::::::::::::----                           
                               .....--------------------------------.                               
                                       .........---------....                               
```
