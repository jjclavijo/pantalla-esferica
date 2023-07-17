import numpy as np
import open3d as o3d
import rasterio
import pyautogui
import configparser
import cv2

from scipy.spatial.transform import Rotation as R
from scipy.spatial import Delaunay
from sys import argv


class Proyectores():


    def __init__(self, archivo_configuracion):

        # lee el archivo de configuracion
        ruta_imagen, resolucion_m, resolucion, tipo_malla, subdivision, textura = self.lee_configuracion(archivo_configuracion)

        # obtiene las posiciones y tamanos de cada ventana, en base a la resolucion
        self.parametros_ventanas = self.ubicaciones_ventanas(resolucion_m, resolucion)

        # esta variable controla si se muestra la escena de calibracion o de la esfera
        self.mostrando_calibracion = False

        # esta variable controla si se realizan grandes o pequenos movimientos
        self.pequenos_movimientos = False 

        # listas de widgets y escenas
        self.widgets = []
        self.scenes = []

        # parametros de orientacion/visualizacion de las camaras
        self.parametros = []

        # estado prendido/apagado de las camaras
        self.pantalla_negra = []

        # inicializa la aplicacion
        self.app = o3d.visualization.gui.Application.instance
        self.app.initialize()

        # crea las geometrias de la escena de presentacion y calibracion
        self.esfera, self.cilindro, self.anillo, self.cubo = self.crear_geometrias(tipo_malla, subdivision)

        # obtiene las coordenadas latlon de los vertices de la esfera
        self.latlon = self.obtener_coordenadas_esfericas()

        if textura:

            # colorea la esfera aplicando textura (no anda)
            self.cargar_textura(ruta_imagen)

        else:

            # colorea la esfera con los datos de la imagen
            self.cargar_colores_vertices(ruta_imagen)

        # agrega las camaras en las posiciones estandar
        posiciones_de_camara = [[ 5,  0, 0],
                                [ 0,  5, 0],
                                [-5,  0, 0],
                                [ 0, -5, 0]]
        
        for pos in posiciones_de_camara:
            self.agregar_camara(pos)

        # activa la primer ventana
        for c in self.parametros_ventanas:
            pyautogui.click(*self.parametros_ventanas[c]['centro'])
        pyautogui.click(*self.parametros_ventanas[0]['centro'])

        # corre la aplicacion
        self.app.run()


    def lee_configuracion(self, archivo_configuracion):

        '''
        funcion para leer el archivo de configuracion
        '''

        config = configparser.ConfigParser()
        config.read(archivo_configuracion)

        ruta_imagen = config['configs']['ruta_imagen']
        resolucion_x_m = int(config['configs']['resolucion_x_m'])
        resolucion_y_m = int(config['configs']['resolucion_y_m'])
        resolucion_x = int(config['configs']['resolucion_x'])
        resolucion_y = int(config['configs']['resolucion_y'])
        tipo_malla = config['configs']['tipo_malla']

        if tipo_malla == 't':
            subdivision = int(config['configs']['subdivision'])
        else:
            subdivision = float(config['configs']['subdivision'])

        resolucion_m = (resolucion_x_m, resolucion_y_m)
        resolucion = (resolucion_x, resolucion_y)

        textura = config['configs'].getboolean('textura')

        return ruta_imagen, resolucion_m, resolucion, tipo_malla, subdivision, textura
    

    def ubicaciones_ventanas(self, resolucion_m, resolucion):

        parametros_ventanas = {}
        for i in range(4):

            # coordenadas del vertice superior izquierdo
            x0 = resolucion_m[0] + resolucion[0] * i
            y0 = 0

            # cada ventana tiene el mismo tamano
            delta_x = resolucion[0]
            delta_y = resolucion[1]

            # coordenadas del centro, para hacer click
            xc = x0 + resolucion[0] / 2
            yc = y0

            parametros_ventanas[i] = {}
            parametros_ventanas[i]['origen'] = [x0, y0]
            parametros_ventanas[i]['deltas'] = [delta_x, delta_y]
            parametros_ventanas[i]['centro'] = [xc, yc]

        return parametros_ventanas

    
    def crear_geometrias(self, tipo, subdivision):


        if tipo == 't':         # si queremos malla triangular

            # crear una esfera de radio 1
            esfera = o3d.geometry.TriangleMesh.create_sphere(1)

            # con esto se puede dividir la malla
            esfera = esfera.subdivide_loop(number_of_iterations=subdivision)
            esfera.compute_vertex_normals()
        
        else:                   # si queremos malla con rectangulos (terminan siendo triangulos)
            
            lat = np.arange(-np.pi / 2, np.pi / 2 + 0.001, np.radians(subdivision))
            lon = np.arange(-np.pi, np.pi + 0.001, np.radians(subdivision))
            lons, lats = np.meshgrid(lon, lat)
            lonlat = np.stack([lons, lats], 0)
            lonlat = lonlat.reshape((2, -1)).T

            triangulacion = Delaunay(lonlat)
            
            x = np.cos(lonlat[:, 1]) * np.cos(lonlat[:, 0])
            y = np.cos(lonlat[:, 1]) * np.sin(lonlat[:, 0])
            z = np.sin(lonlat[:, 1])
            xyz = np.stack([x, y, z], 0).T

            xyz = o3d.utility.Vector3dVector(xyz)
            ind = o3d.utility.Vector3iVector(triangulacion.simplices)

            esfera = o3d.geometry.TriangleMesh(xyz, ind)
            esfera.compute_vertex_normals()


        # ------- crear las geometrias para la calibracion -------

        cilindro = o3d.geometry.TriangleMesh.create_cylinder(radius=0.1, height=4.0)
        cilindro.compute_vertex_normals()
        cilindro.paint_uniform_color([0.9, 0.1, 0.1])

        anillo = o3d.geometry.TriangleMesh.create_torus(torus_radius=0.175, tube_radius=0.075)
        anillo.compute_vertex_normals()
        anillo.paint_uniform_color([0.1, 0.9, 0.1])

        # el cubo lo creo con textura, para que las caras esten coloreadas de manera uniforme
        lado = 0.5
        vertices = [[0, 0, 0], [0, lado, 0], [lado, lado, 0], [lado, 0, 0],
        [0, 0, lado], [0, lado, lado], [lado, lado, lado], [lado, 0, lado]]

        triangulos = [[0, 1, 2], [0, 2, 3], [6, 5, 4], [7, 6, 4], [5, 1, 0], [0, 4, 5],
                [3, 2, 6], [6, 7, 3], [0, 3, 7], [0, 7, 4], [1, 5, 6], [1, 6, 2]]


        cubo = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(triangulos))
        cubo.compute_vertex_normals()

        # carga la textura del cubo
        textura = cv2.imread('cube.png')
        self.material_cubo = o3d.visualization.rendering.MaterialRecord()
        self.material_cubo.albedo_img = o3d.io.read_image('cube.png')

        v_uv = np.random.rand(len(triangulos) * 3, 2)

        dx = 0.25
        dy = 1.00 / 3

        v_uv = np.array([
            [    dx,     dy], [    dx, 2 * dy], [2 * dx, 2 * dy],
            [    dx,     dy], [2 * dx, 2 * dy], [2 * dx,     dy],
            [3 * dx, 2 * dy], [4 * dx, 2 * dy], [4 * dx,     dy],
            [3 * dx,     dy], [3 * dx, 2 * dy], [4 * dx,     dy],
            [     0, 2 * dy], [    dx, 2 * dy], [    dx,     dy],
            [    dx,     dy], [     0,     dy], [     0, 2 * dy],
            [2 * dx,     dy], [2 * dx, 2 * dy], [3 * dx, 2 * dy],
            [3 * dx, 2 * dy], [3 * dx,     dy], [2 * dx,     dy],
            [    dx,     dy], [2 * dx,     dy], [2 * dx,      0],
            [    dx,     dy], [2 * dx,      0], [    dx,      0],
            [    dx, 2 * dy], [    dx, 3 * dy], [2 * dx, 3 * dy],
            [    dx, 2 * dy], [2 * dx, 3 * dy], [2 * dx, 2 * dy]
        ])

        cubo.triangle_uvs = o3d.utility.Vector2dVector(v_uv)
        cubo.textures = [o3d.geometry.Image(textura)]
        cubo.triangle_material_ids = o3d.utility.IntVector([0] * len(triangulos))


        # mueve el cubo y lo rota
        cubo.translate(np.array([-lado / 2, -lado / 2, -lado / 2]))

        a = np.radians(45)
        R = np.array([[1,         0,          0],
                      [0, np.cos(a), -np.sin(a)],
                      [0, np.sin(a),  np.cos(a)]])
        cubo.rotate(R)

        a = np.arctan(1 / np.sqrt(2))
        R = np.array([[np.cos(a), 0, -np.sin(a)],
                      [        0, 1,          0],
                      [np.sin(a), 0,  np.cos(a)]])
        cubo.rotate(R)

        cubo.translate(np.array([0.0, 0.0, 1.5]))


        return esfera, cilindro, anillo, cubo


    def obtener_coordenadas_esfericas(self):


        # Obtener las coordenadas cartesianas de los vértices
        vertices = np.copy(np.asarray(self.esfera.vertices))

        # Calcular las coordenadas esféricas de los vértices
        r = np.sqrt(np.sum(vertices ** 2, axis=1))
        lat = np.sign(vertices[:, 2]) * np.arccos(np.sqrt(np.sum(vertices[:, :2] ** 2, axis=1)) / r)
        lon = np.arctan2(vertices[:, 1], vertices[:, 0])

        # Convertir los valores de lat y lon a grados
        lat = np.degrees(lat)
        lon = np.degrees(lon)

        return np.stack([lat, lon]).T


    def cargar_textura(self, ruta_imagen):

        '''
        funcion para cargar la textura de la esfera
        '''
        
        indices_vertices_de_triangulos = np.asarray(self.esfera.triangles)

        v_uv = []

        for t in range(indices_vertices_de_triangulos.shape[0]):
            for i in range(3):
                index = indices_vertices_de_triangulos[t, i]
                c = self.latlon[index, :]

                u = (c[1] + 180) / 360
                v = (c[0] + 90) / 180

                v_uv.append([u, v])

        v_uv = np.array(v_uv)

        textura = cv2.imread(ruta_imagen)
        self.material_esfera = o3d.visualization.rendering.MaterialRecord()
        self.material_esfera.albedo_img = o3d.io.read_image(ruta_imagen)

        self.esfera.triangle_uvs = o3d.utility.Vector2dVector(v_uv)
        self.esfera.textures = [o3d.geometry.Image(textura)]
        self.esfera.triangle_material_ids = o3d.utility.IntVector([0] * len(indices_vertices_de_triangulos))

    
    def cargar_colores_vertices(self, ruta_imagen):

        '''
        esta funcion obtiene los colores para cada vertice de la malla
        '''

        self.material_esfera = o3d.visualization.rendering.MaterialRecord()

        colores_vertices = []

        with rasterio.open(ruta_imagen) as src:

            for c in self.latlon:

                # transforma latlon al sistema de coordenadas de la imagen
                x, y = src.index(c[1], c[0])

                # lee el color en el pixel x,y
                contenido = src.read(window=((x, x+1), (y, y+1)))

                # primer caso: imagen de 3 bandas
                try:
                    r, g, b = contenido

                    if len(r) == 0 or len(g) == 0 or len(b) == 0:
                        colores_vertices.append([0, 0, 0])
                    else:
                        try:
                            colores_vertices.append([r[0][0], g[0][0], b[0][0]])
                        except:
                            colores_vertices.append([0, 0, 0])
                
                # segundo caso: imagen de una unica banda
                except ValueError:  
                    try:
                        colores_vertices.append([255 - contenido[0][0][0], 0, 0])
                    except IndexError:
                        colores_vertices.append([0, 0, 0])


        # se colorean los vertices en rgb
        self.esfera.vertex_colors = o3d.utility.Vector3dVector(np.array(colores_vertices) / 255.0)
        
    
    def agregar_camara(self, posicion):

        """
        Crear una ventana con una camara en la posición deseada, apuntando hacia el centro.
        Se crea la camara con la escena y el widget correspondiente y se carga la
        geometría.

        Posicion: array de dimensión 3.
        """

        # indice de la ventana que se esta creando
        indice = len(self.widgets) + 1

        # calculo la posicion de la pantalla donde se muestra la ventana
        x0      = self.parametros_ventanas[indice - 1]['origen'][0]
        y0      = self.parametros_ventanas[indice - 1]['origen'][1]
        delta_x = self.parametros_ventanas[indice - 1]['deltas'][0]
        delta_y = self.parametros_ventanas[indice - 1]['deltas'][1]

        # Crear una ventana de tamano delta_x x delta_y pixeles en la posicion x,y
        nombre = f'{indice}'
        w = self.app.create_window(f"Camara {nombre}", delta_x, delta_y, x0, y0)

        # -------------------------------------------------

        # crear la escena (indicando la ventana en que se va a renderizar)
        scene = o3d.visualization.rendering.Open3DScene(w.renderer)

        # Agregar las geometrías (ver si queremos agregar mas geometrias)
        scene.add_geometry('esfera', self.esfera, self.material_esfera)

        scene.add_geometry('cilindro', self.cilindro, o3d.visualization.rendering.MaterialRecord())
        scene.show_geometry('cilindro', False)

        scene.add_geometry('anillo', self.anillo, o3d.visualization.rendering.MaterialRecord())
        scene.show_geometry('anillo', False)

        scene.add_geometry('cubo', self.cubo, self.material_cubo)
        scene.show_geometry('cubo', False)

        # crea el punto que indica si es la escena activa o no
        # (uno para la escena de muestra y otro para la de calibracion)

        punto = o3d.geometry.TriangleMesh.create_sphere(0.075)
        punto.compute_vertex_normals()
        punto.paint_uniform_color([1.0, 0.8, 0.0])
        punto.translate(np.array(posicion) / 5.0)
        
        scene.add_geometry('punto', punto, o3d.visualization.rendering.MaterialRecord())
        if indice != 1:
            scene.show_geometry('punto', False)

        # un cubo negro para cuando se apaga el proyector
        cubo_negro = o3d.geometry.TriangleMesh.create_box(width=2.0, height=2.0, depth=10.0)
        cubo_negro.compute_vertex_normals()
        cubo_negro.paint_uniform_color([0.0, 0.0, 0.0])
        cubo_negro.translate([-1.0, -1.0, -5.0])
        scene.add_geometry('cubo_negro', cubo_negro, o3d.visualization.rendering.MaterialRecord())
        scene.show_geometry('cubo_negro', False)

        # -------------------------------------------------

        # Agregarle el widget contenedor de la escena
        scene_widget = o3d.visualization.gui.SceneWidget()

        # Asigna la escena al widget
        scene_widget.scene = scene

        # Configurar la cámara
        apuntar_a   = np.array([0., 0., 0.], dtype=np.float64)
        direccion_v = np.array([0., 0., 1.], dtype=np.float64)

        # scene_widget.setup_camera(60, scene.bounding_box, apuntar_a)
        scene_widget.look_at(apuntar_a, posicion, direccion_v)

        # agregar las funciones de las teclas al widget
        scene_widget.set_on_key(self.get_on_key(indice - 1))

        # Agregar el widget a la ventana.
        w.add_child(scene_widget)       

        # -------------------------------------------------

        self.widgets.append(scene_widget)
        self.scenes.append(scene)
        self.parametros.append([apuntar_a, posicion, direccion_v])
        self.pantalla_negra.append(False)
        

    
    def get_on_key(self, index):

        """
        Los mapas de teclado se setean por cada widget.
        Como son todos iguales creamos una función generadora de mapas, que crea la
        función adecuada que recibe un evento de teclado y hace lo que tiene que
        hacer.
        """

        def on_key(e):

            """
            Un mapa de teclas para ejecutar las acciones necesarias para el proyecto
            """


            # ------------ para finalizar el programa ESC ------------------

            if e.key == o3d.visualization.gui.KeyName.ESCAPE:
                exit()

            # ------------ para apagar la escena actual O ------------------

            if e.key == o3d.visualization.gui.KeyName.O:

                if e.type == o3d.visualization.gui.KeyEvent.DOWN:
                
                    if self.pantalla_negra[index]:
                        self.scenes[index].set_background([1, 1, 1, 1])
                        self.scenes[index].show_geometry('cubo_negro', False)
                        self.pantalla_negra[index] = False
                    else:
                        self.scenes[index].set_background([0, 0, 0, 1])
                        self.scenes[index].show_geometry('cubo_negro', True)
                        self.pantalla_negra[index] = True

                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED                

            # ------------ para cambiar las camaras desde la 1 a la 4 ----------------------

            if e.key == o3d.visualization.gui.KeyName.Q:
                self.scenes[index].show_geometry('punto', False)
                self.scenes[0].show_geometry('punto', True)
                pyautogui.click(*self.parametros_ventanas[0]['centro'])
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            
            
            if e.key == o3d.visualization.gui.KeyName.W:
                self.scenes[index].show_geometry('punto', False)
                self.scenes[1].show_geometry('punto', True)
                pyautogui.click(*self.parametros_ventanas[1]['centro'])
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            

            if e.key == o3d.visualization.gui.KeyName.E:
                self.scenes[index].show_geometry('punto', False)
                self.scenes[2].show_geometry('punto', True)
                pyautogui.click(*self.parametros_ventanas[2]['centro'])
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            

            if e.key == o3d.visualization.gui.KeyName.R:
                self.scenes[index].show_geometry('punto', False)
                self.scenes[3].show_geometry('punto', True)
                pyautogui.click(*self.parametros_ventanas[3]['centro'])             
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            

            # ------------ para cambiar a la escena de calibracion ----------------------
            
            if e.key == o3d.visualization.gui.KeyName.C:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    if self.mostrando_calibracion:
                        for s in self.scenes:
                            s.show_geometry('anillo', False)
                            s.show_geometry('cilindro', False)
                            s.show_geometry('cubo', False)
                            s.show_geometry('esfera', True)
                        
                        self.mostrando_calibracion = False
                    else:
                        for s in self.scenes:
                            s.show_geometry('anillo', True)
                            s.show_geometry('cilindro', True)
                            s.show_geometry('cubo', True)
                            s.show_geometry('esfera', False)
                        
                        self.mostrando_calibracion = True

                    # con esto se actualizan a mano las ventanas. ver si se puede mejorar despues
                    for c in self.parametros_ventanas:
                        pyautogui.click(*self.parametros_ventanas[c]['centro'])
                    pyautogui.click(*self.parametros_ventanas[index]['centro'])

                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            

            # ------------ para activar y desactivar pequenos movimientos ----------------------

            # P activa y desactiva pequenos movimientos
            if e.key == o3d.visualization.gui.KeyName.P:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    if self.pequenos_movimientos:
                        self.pequenos_movimientos = False
                    else:
                        self.pequenos_movimientos = True
                
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # cambiar esto si los movimientos tienen que ser mas grandes o mas chicos
            if self.pequenos_movimientos:
                angulo = 0.1 * np.pi / 180
                escala = 0.99
                delta  = 0.01
            else:
                angulo = 1.0 * np.pi / 180
                escala = 0.90
                delta  = 0.10


            # --------------- para rotaciones, escalas y movimientos -------------------------

            # L gira sobre el eje de la esfera hacia la derecha
            if e.key == o3d.visualization.gui.KeyName.L:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:
                    
                    apuntar_a = self.parametros[index][0]
                    posicion = self.parametros[index][1]
                    direccion_v = self.parametros[index][2]

                    nuevo_apuntar_a, nueva_posicion, nueva_direccion_v = self.rotar_en_z(angulo, apuntar_a, posicion, direccion_v)
                    
                    self.parametros[index][0] = nuevo_apuntar_a
                    self.parametros[index][1] = nueva_posicion
                    self.parametros[index][2] = nueva_direccion_v

                    self.widgets[index].look_at(*self.parametros[index])
                
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # H gira sobre el eje de la esfera hacia la izquierda
            if e.key == o3d.visualization.gui.KeyName.H:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:
                    
                    apuntar_a = self.parametros[index][0]
                    posicion = self.parametros[index][1]
                    direccion_v = self.parametros[index][2]

                    nuevo_apuntar_a, nueva_posicion, nueva_direccion_v = self.rotar_en_z(-angulo, apuntar_a, posicion, direccion_v)
                    
                    self.parametros[index][0] = nuevo_apuntar_a
                    self.parametros[index][1] = nueva_posicion
                    self.parametros[index][2] = nueva_direccion_v

                    self.widgets[index].look_at(*self.parametros[index])

                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # J gira la vista hacia la arriba
            if e.key == o3d.visualization.gui.KeyName.J:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:
                    
                    posicion = self.parametros[index][1]

                    rotado = self.rotate_about_origin_vertical(angulo, posicion)
                    self.parametros[index][1] = rotado

                    self.widgets[index].look_at(*self.parametros[index])
                    
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # K gira la vista hacia la abajo
            if e.key == o3d.visualization.gui.KeyName.K:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:
                    
                    posicion = self.parametros[index][1]

                    rotado = self.rotate_about_origin_vertical(-angulo, posicion)
                    self.parametros[index][1] = rotado

                    self.widgets[index].look_at(*self.parametros[index])
                    
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # T gira la vista alrededor de la visual, en sentido horario
            if e.key == o3d.visualization.gui.KeyName.T:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    posicion = np.copy(self.parametros[index][1]).astype(np.float64)
                    posicion /= np.linalg.norm(posicion)
                    rot = R.from_rotvec(angulo * posicion)

                    direccion_v = np.copy(self.parametros[index][2]).astype(np.float64)
                    self.parametros[index][2] = rot.apply(direccion_v)

                    self.widgets[index].look_at(*self.parametros[index])
                
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # G gira la vista alrededor de la visual, en sentido anti-horario
            if e.key == o3d.visualization.gui.KeyName.G:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    posicion = np.copy(self.parametros[index][1]).astype(np.float64)
                    posicion /= np.linalg.norm(posicion)
                    rot = R.from_rotvec(-angulo * posicion)

                    direccion_v = np.copy(self.parametros[index][2]).astype(np.float64)
                    self.parametros[index][2] = rot.apply(direccion_v)

                    self.widgets[index].look_at(*self.parametros[index])
                
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # V Acerca la camara en el mismo sentido en el cual esta apuntando
            if e.key == o3d.visualization.gui.KeyName.V:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    posicion = np.copy(self.parametros[index][1]).astype(np.float64)
                    self.parametros[index][1] = posicion * escala

                    self.widgets[index].look_at(*self.parametros[index])
                
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # B Aleja la camara en el mismo sentido en el cual esta apuntando
            if e.key == o3d.visualization.gui.KeyName.B:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    posicion = np.copy(self.parametros[index][1]).astype(np.float64)
                    self.parametros[index][1] = posicion / escala

                    self.widgets[index].look_at(*self.parametros[index])
                
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            
            
            # LEFT mueve hacia la izquierda
            if e.key == o3d.visualization.gui.KeyName.LEFT:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    apuntar_a, posicion, direccion_v = self.parametros[index]

                    aux = posicion - apuntar_a
                    unit = np.cross(direccion_v, aux)
                    unit /= np.linalg.norm(unit)

                    self.parametros[index][0] = apuntar_a + delta * unit
                    self.parametros[index][1] = posicion + delta * unit

                    self.widgets[index].look_at(*self.parametros[index])

                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            
            
            # RIGHT mueve hacia la derecha
            if e.key == o3d.visualization.gui.KeyName.RIGHT:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    apuntar_a, posicion, direccion_v = self.parametros[index]

                    aux = posicion - apuntar_a
                    unit = np.cross(direccion_v, aux)
                    unit /= np.linalg.norm(unit)

                    self.parametros[index][0] = apuntar_a - delta * unit
                    self.parametros[index][1] = posicion - delta * unit

                    self.widgets[index].look_at(*self.parametros[index])

                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            

            # UP mueve hacia arriba
            if e.key == o3d.visualization.gui.KeyName.UP:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    apuntar_a, posicion, direccion_v = self.parametros[index]

                    unit = direccion_v / np.linalg.norm(direccion_v)

                    self.parametros[index][0] = apuntar_a - delta * unit
                    self.parametros[index][1] = posicion - delta * unit

                    self.widgets[index].look_at(*self.parametros[index])

                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            

            # DOWN mueve hacia abajo
            if e.key == o3d.visualization.gui.KeyName.DOWN:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:

                    apuntar_a, posicion, direccion_v = self.parametros[index]

                    unit = direccion_v / np.linalg.norm(direccion_v)

                    self.parametros[index][0] = apuntar_a + delta * unit
                    self.parametros[index][1] = posicion + delta * unit

                    self.widgets[index].look_at(*self.parametros[index])

                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
            

            # ------------ para movimientos del cubo hacia arriba y hacia abajo --------------

            # Z mueve hacia abajo
            if e.key == o3d.visualization.gui.KeyName.Z:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:
                    
                    if self.mostrando_calibracion:

                        for s in self.scenes:
                            s.remove_geometry('cubo')

                        self.cubo.translate(np.array([0.0, 0.0, -delta]))
                        
                        for s in self.scenes:
                            s.add_geometry('cubo', self.cubo, self.material_cubo)

                        pyautogui.click(*self.parametros_ventanas[index]['centro'])
                    
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            # X mueve hacia arriba
            if e.key == o3d.visualization.gui.KeyName.X:
                if e.type == o3d.visualization.gui.KeyEvent.DOWN:
                    
                    if self.mostrando_calibracion:

                        for s in self.scenes:
                            s.remove_geometry('cubo')

                        self.cubo.translate(np.array([0.0, 0.0, delta]))

                        for s in self.scenes:
                            s.add_geometry('cubo', self.cubo, self.material_cubo)

                        pyautogui.click(*self.parametros_ventanas[index]['centro'])
                    
                return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED


            return o3d.visualization.gui.Widget.EventCallbackResult.IGNORED
        

        return on_key


    def rotar_en_z(self, degree, apuntar_a, posicion, direccion_v):

        '''
        funcion para rotar las direcciones en el eje z de la esfera
        '''

        s = posicion - apuntar_a

        eje = np.array([0, 0, 1])
        rot = R.from_rotvec(-degree * eje)

        nuevo_apuntar_a = rot.apply(apuntar_a)
        nueva_posicion = rot.apply(s) + nuevo_apuntar_a
        nueva_direccion_v = rot.apply(direccion_v)

        return nuevo_apuntar_a, nueva_posicion, nueva_direccion_v
    
    
    # def rotate_about_origin_horizontal(self, degree, point):

    #     """
    #     Un helper para rotar alrededor del eje Z

    #     Probablemente esto tenga que convertirse usando R.from_euler
    #     o R.from_vec. Hay que ver si hay que rotar alrededor del eje z o alrededor
    #     de la vertical de la cámara.
    #     """
    #     M = np.array(
    #         [
    #             [np.cos(degree), -np.sin(degree), 0],
    #             [np.sin(degree), np.cos(degree), 0],
    #             [0, 0, 1],
    #         ]
    #     )

    #     return point @ M
    

    def rotate_about_origin_vertical(self, degree, point):

        """
        un helper para crear una matriz de rotación "hacia arriba" o "hacia abajo"

        Nótese que esta aproximación funciona mal muy cerca de los polos, pero
        puede que no pueda ser necesario.

        También podría convertirse usando R.from_vec. Hay que ver si el vector
        alrededor del que hay que rotar es el producto cruz entre la visual y la
        vertical.
        """

        point = np.array(point)

        up_dir = np.cross(np.cross([0, 0, 1], point), point)
        up_dir = np.sign(up_dir[-1]) * up_dir / np.linalg.norm(up_dir)

        scale = np.linalg.norm(point)

        offset = np.tan(degree) * scale

        new_point = point + up_dir * offset

        new_scale = np.linalg.norm(new_point)

        new_point = new_point * scale / new_scale

        return new_point

    

if __name__ == "__main__":

    archivo_configuracion = argv[1]

    p = Proyectores(archivo_configuracion)

