import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui

def crear_esfera():
    """
    Crear una esfera pintada
    """

    # Crear una esfera de radio 1
    esfera = o3d.geometry.TriangleMesh.create_sphere(1)
    # Internet dice que esto sirve para que se vea bien.
    esfera.compute_vertex_normals()

    # Setear los colores de la esfera. Acá podría cargarse
    # consultando por posición los datos de un mapa.
    esfera.vertex_colors = o3d.utility.Vector3dVector(
        np.array(
            [
                np.linspace(0, 30, 762) % 1,
                (np.linspace(0, 30, 762) % 1).reshape(6, -1).T.flatten(),
                np.linspace(1, 0, 762),
            ]
        ).T
    )
    return esfera

# ---------------------
# GLOBALS
# ---------------------

# Aunque no nos guste usar globals, es la primera forma que encontré de
# tener disponibles los datos de las cuatro cámaras y compartir la geometría
# representada.
# Lo lógico sería armar una clase o diccionario contenedor y ir pasandolo entre
# llamadas a funciones. Así también se puede hacer pickle de eso para guardar
# proyectos ya creados y calibrados.

geometrias = []
windows = []
look_at_list = []
vertical_orientation_list = []
widgets = []
scenes = []
camera_angle_list = []
move_angle = 1 * np.pi / 180

# ----------------------
# Hasta acá los Globals
# ----------------------

def agregar_camara(posicion,nombre=''):
    """
    Crear una ventana con una camara en la posición deseada, apuntando hacia el centro.
    Se crea la camara con la escena y el widget correspondiente y se carga la
    geometría.

    Posicion: array de dimensión 3.
    """

    # Traer los globals
    global windows
    global widgets
    global scenes
    global look_at_list

    if not nombre:
        nombre = f'{len(windows)+1}'

    # Crear una ventana
    w = gui.Application.instance.create_window(f"Camara {nombre}", 640, 480)
    # Agregarle el widget contenedor de la escena
    scene_widget = gui.SceneWidget()
    # crear la escena (indicando la ventana en que se va a renderizar)
    scene = o3d.visualization.rendering.Open3DScene(w.renderer)

    # Agregar las geometrías
    for g in geometrias:
        scene.add_geometry(*g)

    # Asigna la escena al widget
    scene_widget.scene = scene

    # Configurar la cámara
    scene_widget.setup_camera(60, scene.bounding_box, (0, 0, 0)) #???
    scene_widget.look_at((0,0,0),posicion,(0,0,1))

    # Agregar el widget a la ventana.
    w.add_child(scene_widget)


    # Agregar los objetos a las listas globales.
    windows.append(w)
    widgets.append(scene_widget)
    scenes.append(scene)
    look_at_list.append(posicion)
    vertical_orientation_list.append(np.array([0.,0.,1.]))
    camera_angle_list.append(60)

    return 0

from scipy.spatial.transform import Rotation as R

def rotate_about_origin_horizontal(degree,point):
    """
    Un helper para rotar alrededor del eje Z

    Probablemente esto tenga que convertirse usando R.from_euler
    o R.from_vec. Hay que ver si hay que rotar alrededor del eje z o alrededor
    de la vertical de la cámara.
    """
    M = np.array(
        [
            [np.cos(degree), -np.sin(degree), 0],
            [np.sin(degree), np.cos(degree), 0],
            [0, 0, 1],
        ]
    )

    return point @ M

def rotate_about_origin_vertical(degree,point):
    """
    un helper para crear una matriz de rotación "hacia arriba" o "hacia abajo"

    Nótese que esta aproximación funciona mal muy cerca de los polos, pero
    puede que no pueda ser necesario.

    También podría convertirse usando R.from_vec. Hay que ver si el vector
    alrededor del que hay que rotar es el producto cruz entre la visual y la
    vertical.
    """

    point = np.array(point)

    up_dir = np.cross(np.cross([0,0,1],point),point)
    up_dir = np.sign(up_dir[-1]) * up_dir / np.linalg.norm(up_dir)

    scale = np.linalg.norm(point)

    offset = np.tan(degree)*scale

    new_point = point + up_dir * offset

    new_scale = np.linalg.norm(new_point)

    new_point = new_point * scale / new_scale

    return new_point


def get_on_key(index):
    """
    Los mapas de teclado se setean por cada widget.
    Como son todos iguales creamos una función generadora de mapas, que crea la
    función adecuada que recibe un evento de teclado y hace lo que tiene que
    hacer.
    """
    def on_key(e):
        """
        Un mapa de teclas para ejecutar las acciones necesarias para el
        proyecto. A hoy lo que hace es rotar la camara alrededor del centro de
        la esfera con H o L, y cambiar la distancia de la cámara con J y K,
        """

        # Cargar los globals, a los que vamos a acceder por índice.
        global look_at_list
        global vertical_orientation_list
        global widgets
        global move_angle

        # Shift permite movimientos mas largos o más cortos
        # La variable es move_angle y está en radianes.
        # Para el ángulo de visual de la cámara se la convierte a grados.
        # Para alejar/acercar la cámara se convertirá a 10cm / 1cm
        if e.key == gui.KeyName.LEFT_SHIFT:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                move_angle = 1 * np.pi / 180
            if e.type == gui.KeyEvent.DOWN:  # check UP so we default to DOWN
                move_angle = 0.1 * np.pi / 180

        # Control permite movimientos mas largos o más cortos
        # La variable es move_angle y está en radianes.
        # Para el ángulo de visual de la cámara se la convierte a grados.
        # Para alejar/acercar la cámara se convertirá a 10cm / 1m
        if e.key == gui.KeyName.LEFT_CONTROL:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                move_angle = 1 * np.pi / 180
            if e.type == gui.KeyEvent.DOWN:  # check UP so we default to DOWN
                move_angle = 10 * np.pi / 180

        # L gira la vista hacia la derecha
        if e.key == gui.KeyName.L:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en un sentido.
                original = look_at_list[index]
                rotado = rotate_about_origin_horizontal(move_angle,
                                                         original)
                look_at_list[index] = rotado

                # Actualizar la cámara.
                widgets[index].look_at((0, 0, 0), look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # H gira la vista hacia la izquierda
        if e.key == gui.KeyName.H:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en el otro sentido.
                original = look_at_list[index]
                rotado = rotate_about_origin_horizontal(-move_angle,
                                                         original)
                look_at_list[index] = rotado

                # Actualizar la cámara.
                widgets[index].look_at( (0, 0, 0),look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # J gira la vista hacia la arriba
        if e.key == gui.KeyName.J:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en un sentido.
                original = look_at_list[index]
                rotado = rotate_about_origin_vertical(move_angle,
                                                         original)
                look_at_list[index] = rotado

                # Actualizar la cámara.
                widgets[index].look_at((0, 0, 0), look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # K gira la vista hacia la abajo
        if e.key == gui.KeyName.K:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en el otro sentido.
                original = look_at_list[index]
                rotado = rotate_about_origin_vertical(-move_angle,
                                                         original)
                look_at_list[index] = rotado

                # Actualizar la cámara.
                widgets[index].look_at( (0, 0, 0),look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # T gira la vista alrededor de la visual, en sentido horario
        if e.key == gui.KeyName.T:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en un sentido.
                axis = np.array(look_at_list[index],dtype=float)
                axis /= np.linalg.norm(axis)
                rot = R.from_rotvec(move_angle * axis)

                vo = vertical_orientation_list[index]

                vertical_orientation_list[index] = rot.apply(vo)

                # Actualizar la cámara.
                widgets[index].look_at((0, 0, 0), look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # G gira la vista alrededor de la visual, en sentido anti-horario
        if e.key == gui.KeyName.G:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Rotar en el otro sentido.
                axis = np.array(look_at_list[index],dtype=float)
                axis /= np.linalg.norm(axis)
                rot = R.from_rotvec(-move_angle * axis)

                vo = vertical_orientation_list[index]

                vertical_orientation_list[index] = rot.apply(vo)

                # Actualizar la cámara.
                widgets[index].look_at((0, 0, 0), look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # V Acerca la camara
        if e.key == gui.KeyName.V:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Alejar la cámara
                unit = move_angle * 18 / np.pi
                lookat = np.array(look_at_list[index])
                look_at_list[index] = lookat + unit * lookat / np.linalg.norm(lookat)
                widgets[index].look_at( (0, 0, 0),look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # B Aleja la camara
        if e.key == gui.KeyName.B:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                # Acercar la cámara
                unit = move_angle * 18 / np.pi
                lookat = np.array(look_at_list[index])
                look_at_list[index] = lookat - unit * lookat / np.linalg.norm(lookat)
                widgets[index].look_at( (0, 0, 0),look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # N Amplía el ángulo de la cámara
        if e.key == gui.KeyName.N:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                ca = camera_angle_list[index] + move_angle * (180 / np.pi)
                camera_angle_list[index] = ca

                widgets[index].setup_camera(camera_angle_list[index], scenes[index].bounding_box, (0, 0, 0)) #???
                widgets[index].look_at( (0, 0, 0),look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        # M Reduce el ángulo de la cámara
        if e.key == gui.KeyName.M:
            if e.type == gui.KeyEvent.UP:  # check UP so we default to DOWN
                ca = camera_angle_list[index] - move_angle * (180 / np.pi)
                camera_angle_list[index] = ca

                widgets[index].setup_camera(camera_angle_list[index], scenes[index].bounding_box, (0, 0, 0)) #???
                widgets[index].look_at( (0, 0, 0),look_at_list[index],
                                       vertical_orientation_list[index])
            return gui.Widget.EventCallbackResult.HANDLED

        return gui.Widget.EventCallbackResult.IGNORED
    return on_key

if __name__ == "__main__":
    gui.Application.instance.initialize()

    # Creamos la esfera.
    geometrias.append( ('esfera',crear_esfera(),o3d.visualization.rendering.MaterialRecord()))

    # En el ejemplo creamos cuatro cámaras.
    posiciones_de_camara = [ [ 10,  0,0] ,
                             [  0, 10,0] ,
                             [-10,  0,0] ,
                             [  0,-10,0] ]

    for ix,pos in enumerate(posiciones_de_camara):
        agregar_camara(pos)
        widgets[ix].set_on_key(get_on_key(ix))

    gui.Application.instance.run()
