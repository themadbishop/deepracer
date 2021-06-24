import math

def _clamp(value, limits):
    lower, upper = limits
    if value is None:
        return None
    elif (upper is not None) and (value > upper):
        return upper
    elif (lower is not None) and (value < lower):
        return lower
    return value

class PID(object):
    """A simple PID controller."""

    def __init__(
        self,
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        setpoint=0,
        sample_time=0.01,
        output_limits=(None, None),
        auto_mode=True,
        proportional_on_measurement=False,
        error_map=None,
    ):
        """
        Initialize a new PID controller.
        :param Kp: The value for the proportional gain Kp
        :param Ki: The value for the integral gain Ki
        :param Kd: The value for the derivative gain Kd
        :param setpoint: The initial setpoint that the PID will try to achieve
        :param sample_time: The time in seconds which the controller should wait before generating
            a new output value. The PID works best when it is constantly called (eg. during a
            loop), but with a sample time set so that the time difference between each update is
            (close to) constant. If set to None, the PID will compute a new output value every time
            it is called.
        :param output_limits: The initial output limits to use, given as an iterable with 2
            elements, for example: (lower, upper). The output will never go below the lower limit
            or above the upper limit. Either of the limits can also be set to None to have no limit
            in that direction. Setting output limits also avoids integral windup, since the
            integral term will never be allowed to grow outside of the limits.
        :param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
        :param proportional_on_measurement: Whether the proportional term should be calculated on
            the input directly rather than on the error (which is the traditional way). Using
            proportional-on-measurement avoids overshoot for some types of systems.
        :param error_map: Function to transform the error value in another constrained value.
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self._min_output, self._max_output = None, None
        self._auto_mode = auto_mode
        self.proportional_on_measurement = proportional_on_measurement
        self.error_map = error_map

        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = None
        self._last_output = None
        self._last_input = None

        self.output_limits = output_limits
        self.reset()

    def __call__(self, input_, dt=None):
        """
        Update the PID controller.
        Call the PID controller with *input_* and calculate and return a control output if
        sample_time seconds has passed since the last update. If no new output is calculated,
        return the previous output instead (or None if no value has been calculated yet).
        :param dt: If set, uses this value for timestep instead of real time. This can be used in
            simulations when simulation time is different from real time.
        """
        if not self.auto_mode:
            return self._last_output

        dt = 1
        #print( "input: {}".format(input_))
        # Compute error terms
        error = self.setpoint - input_
        #print( "error: {}".format(error))
        d_input = input_ - (self._last_input if (self._last_input is not None) else input_)

        # Check if must map the error
        if self.error_map is not None:
            error = self.error_map(error)

        # Compute the proportional term
        if not self.proportional_on_measurement:
            # Regular proportional-on-error, simply set the proportional term
            self._proportional = self.Kp * error
        else:
            # Add the proportional error on measurement to error_sum
            self._proportional -= self.Kp * d_input

        # Compute integral and derivative terms
        self._integral += self.Ki * error * dt
        self._integral = _clamp(self._integral, self.output_limits)  # Avoid integral windup

        self._derivative = -self.Kd * d_input / dt

        # Compute final output
        output = self._proportional + self._integral + self._derivative
        output = _clamp(output, self.output_limits)

        # Keep track of state
        self._last_output = output
        self._last_input = input_
        #print( "here: {}".format(output) )
        return output

    def __repr__(self):
        return (
            '{self.__class__.__name__}('
            'Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, '
            'setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, '
            'output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, '
            'proportional_on_measurement={self.proportional_on_measurement!r},'
            'error_map={self.error_map!r}'
            ')'
        ).format(self=self)

    @property
    def components(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
        for visualizing what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
        return self.Kp, self.Ki, self.Kd

    @tunings.setter
    def tunings(self, tunings):
        """Set the PID tunings."""
        self.Kp, self.Ki, self.Kd = tunings

    @property
    def auto_mode(self):
        """Whether the controller is currently enabled (in auto mode) or not."""
        return self._auto_mode

    @auto_mode.setter
    def auto_mode(self, enabled):
        """Enable or disable the PID controller."""
        self.set_auto_mode(enabled)

    def set_auto_mode(self, enabled, last_output=None):
        """
        Enable or disable the PID controller, optionally setting the last output value.
        This is useful if some system has been manually controlled and if the PID should take over.
        In that case, disable the PID by setting auto mode to False and later when the PID should
        be turned back on, pass the last output variable (the control variable) and it will be set
        as the starting I-term when the PID is set to auto mode.
        :param enabled: Whether auto mode should be enabled, True or False
        :param last_output: The last output, or the control variable, that the PID should start
            from when going from manual mode to auto mode. Has no effect if the PID is already in
            auto mode.
        """
        if enabled and not self._auto_mode:
            # Switching from manual mode to auto, reset
            self.reset()

            self._integral = last_output if (last_output is not None) else 0
            self._integral = _clamp(self._integral, self.output_limits)

        self._auto_mode = enabled

    @property
    def output_limits(self):
        """
        The current output limits as a 2-tuple: (lower, upper).
        See also the *output_limits* parameter in :meth:`PID.__init__`.
        """
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set the output limits."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits

        if (None not in limits) and (max_output < min_output):
            raise ValueError('lower limit must be less than upper limit')

        self._min_output = min_output
        self._max_output = max_output

        self._integral = _clamp(self._integral, self.output_limits)
        self._last_output = _clamp(self._last_output, self.output_limits)

    def reset(self):
        """
        Reset the PID controller internals.
        This sets each term to 0 as well as clearing the integral, the last output and the last
        input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._integral = _clamp(self._integral, self.output_limits)

        self._last_output = None
        self._last_input = None

def circle_radius(coords):
    # Flatten the list and assign to variables
    x1, y1, x2, y2, x3, y3 = [i for sub in coords for i in sub]
    a = x1*(y2-y3) - y1*(x2-x3) + x2*y3 - x3*y2
    b = (x1**2+y1**2)*(y3-y2) + (x2**2+y2**2)*(y1-y3) + (x3**2+y3**2)*(y2-y1)
    c = (x1**2+y1**2)*(x2-x3) + (x2**2+y2**2)*(x3-x1) + (x3**2+y3**2)*(x1-x2)
    d = (x1**2+y1**2)*(x3*y2-x2*y3) + (x2**2+y2**2) * (x1*y3-x3*y1) + (x3**2+y3**2)*(x2*y1-x1*y2)
    # In case a is zero (so radius is infinity)
    try:
        r = abs((b**2+c**2-4*a*d) / abs(4*a**2)) ** 0.5
    except:
        r = 999
    return r

speedLookahead = 5
mylaststep = 0
pidset = 0

def calcBestSpeed( speedWaypoints, waypoints ):
    global speedLookahead
    speedlist = []
    for x in range(speedLookahead):
        crad = circle_radius([ waypoints[speedWaypoints[x+y]] for y in range(3) ])
        optimalspeed = math.sqrt(crad)*1.65
        speedlist.append(min([5,optimalspeed]))
        bestSpeed = 5
    return min(speedlist)

def calcbearing(this, last):
    direction = math.atan2(this[0] - last[0], this[1] - last[1])
    track_direction = (math.pi/2 - direction)*180/math.pi
    if track_direction > 180:
        track_direction = track_direction - 360
    return track_direction

dfcpid = PID(1, 0.25, .1, setpoint=0)
w1pid = PID(1, 0.1, .25, setpoint=0)
spid = PID(1, 0.5, .25, setpoint=0)

def setpids():
    global dfcpid, w1pid, w2pid, w3pid, spid
    dfcpid = PID(1, 0.25, .1, setpoint=0)
    w1pid = PID(1, 0.1, .25, setpoint=0)
    spid = PID(1, 0.5, .25, setpoint=0)

setpids()

def reward_function(params):
    global pidset, dfcpid, w1pid, w2pid, w3pid, spid, speedLookahead, mylaststep
    if params['steps'] <= mylaststep or pidset==0: 
        setpids()
        pidset = pidset + 1
    mylaststep = params['steps']
    printoutput = { "team": "cloudhawk" }
    printoutput['params'] = params
    half_width = .5*params['track_width']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    speedWaypoints = [ (closest_waypoints[0]+x)%len(waypoints) for x in range(speedLookahead+2) ]
    bestspeed = calcBestSpeed(speedWaypoints, waypoints)
    last = waypoints[closest_waypoints[0]]
    next1 = waypoints[closest_waypoints[1]]
    next2 = waypoints[(closest_waypoints[1]+1)%len(waypoints)]
    next3 = waypoints[(closest_waypoints[1]+2)%len(waypoints)]
    heading = params['heading']

    # Initialize the reward with typical value
    reward = .2

    w1direction = calcbearing( next1, last )
    w1error = heading-w1direction
    w2direction = calcbearing( next2, last )
    w2error = heading-w2direction
    w3direction = calcbearing( next3, last )
    w3error = heading-w3direction
    dfcerror = params['distance_from_center']
    if params['is_left_of_center']: dfcerror = -dfcerror
    serror = (bestspeed - params['speed'])/5
    errorjson = {
        'dfce' : dfcerror,
        'w1e' : w1error,
        'se' : serror,
        'h' : heading,
        'w1' : w1direction
    }
    printoutput['errors'] = errorjson
    dfcpidreturn = 1/(1 + abs( dfcpid( dfcerror ) ))
    w1pidreturn = 1/(1 + abs( w1pid( w1error ) ))
    spidreturn = 1/(1 + abs( spid(serror) ))
    rewardjson = {
        "dfcp" : dfcpidreturn,
        "w1p" : w1pidreturn,
        "sp" : spidreturn
    }
    printoutput['rewards'] = rewardjson
    printoutput['pidset'] = pidset
    reward = .25 + .75*( dfcpidreturn * w1pidreturn * spidreturn )
    if params['distance_from_center'] > half_width or params['is_offtrack']: reward = 0
    if abs(bestspeed - params['speed']) > 1: reward=0
    if abs(heading - w2direction) > 30: reward=0

    printoutput['finalreward'] = reward
    print(printoutput)

    return float(reward)

reward_function( {'all_wheels_on_track': True, 'x': 3.1997327938476117, 'y': 0.6831032114197831, 'heading': 0.18326595495146542, 'distance_from_center': 0.0, 'projection_distance': 0.14000000000000004, 'progress': 0.7905513581379272, 'steps': 2.0, 'speed': 0.6666666666666666, 'steering_angle': 30.0, 'track_width': 0.7619998019496897, 'track_length': 17.709159380834848, 'waypoints': [(3.059733510017395, 0.6826554089784622), (3.2095088958740234, 0.6831344813108444), (3.359275460243225, 0.6833638250827789), (3.5090349912643433, 0.6834017932415009), (3.6587949991226196, 0.6834610402584076), (3.808555006980896, 0.6835170090198517), (3.9583150148391724, 0.6835691034793854), (4.1080756187438965, 0.6836211383342743), (4.2578349113464355, 0.6836741119623184), (4.407594919204712, 0.683727964758873), (4.557354927062988, 0.6837812215089798), (4.7071144580841064, 0.6838362663984299), (4.856873989105225, 0.6838938742876053), (5.006633043289185, 0.6839521080255508), (5.156393527984619, 0.6840048730373383), (5.306154489517212, 0.6840500980615616), (5.455911874771118, 0.6841173022985458), (5.605645418167114, 0.6843366473913193), (5.75542140007019, 0.6842880994081497), (5.905304670333862, 0.6835954934358597), (6.055286169052124, 0.6823406517505646), (6.204955101013184, 0.6861690580844879), (6.354061603546143, 0.6985173225402832), (6.502514362335205, 0.7188082784414291), (6.643739938735962, 0.7683110386133194), (6.77488899230957, 0.8412670791149139), (6.89846134185791, 0.9262270629405975), (7.0100367069244385, 1.0257667303085327), (7.0997467041015625, 1.1460862159729004), (7.172473669052124, 1.2770325541496277), (7.230445146560669, 1.4172040224075317), (7.272417068481445, 1.565867006778717), (7.283682584762573, 1.7152734994888306), (7.265743970870972, 1.8636599779129024), (7.233960151672363, 2.010729968547821), (7.1842029094696045, 2.154710531234741), (7.114001989364624, 2.2871004343032837), (7.0233659744262695, 2.406221032142639), (6.917426347732544, 2.512663960456848), (6.79807996749878, 2.604923009872436), (6.6672019958496085, 2.6775895357131962), (6.526654481887817, 2.729645013809204), (6.380491495132446, 2.7596704959869385), (6.229795932769775, 2.7700384855270386), (6.079286813735961, 2.7733629941940308), (5.929529666900635, 2.7721140384674072), (5.7797839641571045, 2.7707979679107666), (5.630027532577515, 2.769605040550232), (5.48030161857605, 2.7690484523773193), (5.330573081970215, 2.768457531929016), (5.180745601654053, 2.765363574028015), (5.031071662902832, 2.766121029853821), (4.8823630809783936, 2.7846319675445557), (4.735179901123047, 2.821260929107666), (4.596354961395264, 2.878996968269348), (4.471064329147339, 2.959028959274292), (4.358901500701904, 3.0601580142974854), (4.255730390548706, 3.1701360940933228), (4.16035795211792, 3.2856805324554443), (4.066727519035339, 3.4024704694747925), (3.9719725847244263, 3.518454909324646), (3.8773505687713623, 3.6345274448394775), (3.7827706336975098, 3.7506459951400757), (3.6881529092788696, 3.86673903465271), (3.5935609340667725, 3.9826358556747437), (3.4988315105438232, 4.09949803352356), (3.4035515785217285, 4.217398405075073), (3.294981002807617, 4.319329500198364), (3.1679095029830933, 4.398614168167114), (3.0387414693832397, 4.461370468139648), (2.854969024658203, 4.497744560241699), (2.797850012779234, 4.495018482208252), (2.633301019668579, 4.497664451599121), (2.4294214248657227, 4.4980690479278564), (2.2890069484710693, 4.492910385131836), (2.1444239616394043, 4.488077163696289), (1.99241304397583, 4.483960390090942), (1.842801034450531, 4.479875564575195), (1.6925734877586365, 4.4749414920806885), (1.539882481098175, 4.468656063079834), (1.3862689733505262, 4.457833528518677), (1.2433670163154602, 4.418424367904663), (1.1135604083538055, 4.345951080322266), (0.9965091645717638, 4.250534892082216), (0.8920779228210449, 4.136229991912842), (0.8050850629806519, 4.006568551063538), (0.7456648498773575, 3.8689799308776855), (0.7141403257846834, 3.723703503608705), (0.7072480469942093, 3.572937488555908), (0.714956521987915, 3.4234429597854614), (0.7365620285272598, 3.275694489479065), (0.7720642238855366, 3.129692554473875), (0.8129126578569412, 2.9843615293502808), (0.8494300991296768, 2.838486909866333), (0.8816098272800446, 2.692067503929138), (0.9119606614112854, 2.5454180240631104), (0.942350447177887, 2.3987735509872437), (0.9727316200733185, 2.2521289587020874), (1.0031171143054962, 2.1054846048355103), (1.0335085093975067, 1.958836555480957), (1.063848465681076, 1.8122150301933289), (1.0942798256874084, 1.6655445098876953), (1.125132828950882, 1.518646478652954), (1.1569859981536865, 1.3717305064201355), (1.1986910104751587, 1.2280805110931396), (1.2531161606311798, 1.0885401666164398), (1.3394269943237305, 0.9674179255962372), (1.440102458000183, 0.8561052978038788), (1.5720524787902832, 0.7863914519548416), (1.7143170237541199, 0.7385813295841217), (1.862565040588379, 0.7073544710874557), (2.011545956134796, 0.6859170347452164), (2.1608630418777466, 0.6737564653158188), (2.3105164766311646, 0.6708721071481705), (2.4604655504226685, 0.6761422604322433), (2.610395073890686, 0.6808701455593109), (2.760238528251648, 0.6832202970981598), (2.909994959831238, 0.6831925511360168), (3.059733510017395, 0.6826554089784622)], 'closest_waypoints': [0, 1], 'is_left_of_center': True, 'is_reversed': False, 'closest_objects': [0, 0], 'objects_location': [], 'objects_left_of_center': [], 'object_in_camera': False, 'objects_speed': [], 'objects_heading': [], 'objects_distance_from_center': [], 'objects_distance': [], 'is_crashed': False, 'is_offtrack': False} )
