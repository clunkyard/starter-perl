# A Perl robot on the core binding supplied by the arena.
# Contacts narrow the beam and lead shots from a line fitted through recent
# sightings; remembered passive cues guide the search.
# Bearing is earned: a scan contact's bearing is off by up to its
# bearing_error (degrees, a fixed share of the arc), so a wide beam finds a
# robot and a narrow one aims at it. Contacts within 120 units are exact.
# Clunkyard robot API 2.0.0: one core under every language, the same
# answers everywhere. Facts (a call shows its arguments):
# match: mode ruleset control robots is_3d tick_rate max_ticks
#   robot_id(slot)
# clock: tick seconds remaining_ticks
# arena: size gravity terrain_kind ground_height(x y)
# me: health position velocity heading pitch roll angular_velocity
#   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
#   from_mount(heading pitch) mount_toward(point)
# battery: charge capacity regen sustainable_drive compute
#   drive_cost(fraction)
# drive: hover_lift
# gun: aim_now cooldown_ticks spread max_power damage_scale
#   can_fire(power reserve) shot_speed(power) shot_cost(power)
#   reload_ticks(power) shot_damage(power) power_for_speed(speed)
#   power_for_flight_time(range seconds) lead(contact power)
#   lead_point(position velocity power)
# scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
# sensors: contacts nearest cues contact(slot) position_of(contact)
# tracker: cue nearest_bearing_rate velocity(slot) position(slot)
#   history(slot) last_seen(slot)
# nav: zone zone.margin(point) zone.contains(point margin)
#   zone.distance_from(point) storm storm.margin(point)
#   storm.contains(point margin) storm.distance_from(point) hill
#   hill.margin(point) hill.contains(point margin)
#   hill.distance_from(point) pickups nearest_pickup(kind)
#   field_distance(a b)
# score: me best armour
# angles: normalize(degrees) angle_diff(target current)
#   bearing_to(from to) elevation_to(from to) direction(heading pitch)
#   distance(a b) to_chassis(heading pitch attitude)
#   from_chassis(heading pitch attitude) clamp(value low high)
# raw: number(path) text(path) has(path) flag(path) count(path) line()
# Controls: drive.set(throttle turn pitch_turn)
#   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
#   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
#   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
#   status.set(text value) debug.log(line) raw.extra(json)
# Every control argument is optional; a second call keeps what the first
# set. Absent facts are your language's none. Names are snake_case here;
# your language cases them its own way. Angles are degrees from the +x
# axis; pitch is positive up.
# Manual: https://clunkyard.com/manual#sdk
use strict;
use warnings;
use Clunkyard;
package Starter;
use parent -norequire, 'Clunkyard::Robot';
use constant {
  SEARCH_ARC=>40,  # degrees: sweeps the circle in half a second
  HOLD_WIDTH=>60,  # units across at the target: it stays in the beam from tick to tick
  MIN_ARC=>6,      # degrees: the narrowest the beam holds
  WINDOW=>12,      # ticks of sightings the aim averages
  SIGHTINGS=>8,    # narrow sightings in the window before a shot
  LOOK_EVERY=>40,  # ticks between looks around the target, for a nearer robot
  REACQUIRE=>10,   # ticks to look again where a lost target was
};
sub steer {Clunkyard::angles()->clamp(Clunkyard::angles()->angle_diff($_[0],$_[1])/45)}
sub on_start {my($self)=@_;$self->{free}=$self->match->is_3d && $self->arena->gravity->{z}==0;@$self{qw(dodges dodge_end slot)}=(0,0,0);}
# A straight line through the narrowest of the last WINDOW ticks' sightings:
# where the robot is now, how fast it moves, and how many sightings it rests
# on. Each sighting is off by its bearing_error; the line averages that out.
sub fit {
  my($self,$slot,$tick)=@_;
  my@recent=grep {$_->{tick}>$tick-WINDOW} @{$self->tracker->history($slot)};
  return if @recent<2;
  my($best)=sort {$a<=>$b} map {$_->{bearing_error}} @recent;
  my@seen=grep {$_->{bearing_error}<=1.5*$best} @recent;my$n=@seen;
  return if $n<2;
  my$t0=0;$t0+=$_->{tick}/$n for @seen;
  my$spread=0;$spread+=($_->{tick}-$t0)**2 for @seen;
  my(%position,%velocity);
  for my$axis(qw(x y z)){
    my$mean=0;$mean+=$_->{position}{$axis}/$n for @seen;
    my$slope=0;$slope+=($_->{tick}-$t0)*($_->{position}{$axis}-$mean) for @seen;$slope/=$spread;
    $position{$axis}=$mean+$slope*($tick-$t0);$velocity{$axis}=$slope*$self->match->tick_rate;
  }
  return(\%position,\%velocity,$n);
}
sub on_turn {
  my($self)=@_;my($me,$gun,$scanner,$angles)=($self->me,$self->gun,$self->scanner,Clunkyard::angles());
  $self->drive->forces(brake=>0.2);
  my($target,$cue)=($self->sensors->nearest,$self->tracker->cue);
  my$tick=$self->clock->tick;
  my$last_seen=$self->{slot}?$self->tracker->last_seen($self->{slot}):undef;
  if($target){
    $self->{slot}=$target->{slot};
    my$throttle=$target->{range}>350?0.7:$target->{range}<200?-0.4:0.2;
    $self->drive->set(throttle=>$throttle,turn=>steer($target->{bearing},$me->heading),pitch_turn=>steer($target->{elevation},$me->pitch));
    # Halve the beam each tick until it is HOLD_WIDTH across at the target:
    # narrow enough for a precise bearing, wide enough to keep it in.
    my$hold=2*atan2(HOLD_WIDTH/2,$target->{range})*180/3.14159265358979;$hold=MIN_ARC if $hold<MIN_ARC;
    # In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks, widen
    # it for a tick: a nearer robot it finds becomes the target.
    my$half=$scanner->last->{arc}/2;$half=SEARCH_ARC if $half>SEARCH_ARC;
    my$arc=$tick%LOOK_EVERY==0?SEARCH_ARC:$half>$hold?$half:$hold;
    $scanner->set(heading=>$target->{bearing},arc=>$arc,pitch=>$target->{elevation});
    my$power=$gun->power_for_flight_time($target->{range},1);
    # An exact contact: gun.lead is the intercept if the target holds course.
    # A noisy one: lead the fitted line, and fire only once it rests on
    # enough sightings.
    my($aim,$ready)=(undef,1);
    if($target->{bearing_error}>0){
      my($position,$velocity,$count)=$self->fit($target->{slot},$tick);
      $aim=$position&&$gun->lead_point($position,$velocity,$power);
      $ready=$position&&$count>=SIGHTINGS;
    }else{$aim=$gun->lead($target,$power);}
    my($heading,$pitch)=$aim?($aim->{heading},$aim->{pitch}):($target->{bearing},$target->{elevation});
    $gun->aim(heading=>$heading,pitch=>$pitch);my$now=$gun->aim_now;
    $gun->fire(power=>$power)if $ready && abs($angles->angle_diff($heading,$now->{heading}))<3 && abs($pitch-$now->{pitch})<3 && $gun->can_fire($power,10);
    $self->status->set(text=>$ready?'TRACK':'NARROW',value=>$target->{range});
  }elsif(defined$last_seen && $tick-$last_seen<=REACQUIRE){
    # Lost it: look again where it was last seen, widening the beam, before searching.
    my$where=$self->tracker->position($self->{slot});
    my$bearing=$angles->bearing_to($me->position,$where);
    my$elevation=$self->{free}?$angles->elevation_to($me->position,$where):0;
    my$arc=$scanner->last->{arc}*2;$arc=SEARCH_ARC if $arc>SEARCH_ARC;
    $self->drive->set(throttle=>0.5,turn=>steer($bearing,$me->heading),pitch_turn=>steer($elevation,$me->pitch));
    $scanner->set(heading=>$bearing,arc=>$arc,pitch=>$elevation);
    $self->status->set(text=>'REACQUIRE');
  }elsif($cue){
    my($bearing,$distance)=($angles->bearing_to($me->position,$cue->{position}),$angles->distance($me->position,$cue->{position}));
    my$elevation=$self->{free}?$angles->elevation_to($me->position,$cue->{position}):0;
    my$error=$cue->{bearing_error}<30?$cue->{bearing_error}:30;
    my$across=($tick%5-2)/2*$error;my$up=$self->{free}?(int($tick/5)%3-1)*$error/2:0;
    $self->drive->set(throttle=>0.7,turn=>steer($bearing,$me->heading),pitch_turn=>steer($elevation,$me->pitch));
    $scanner->set(heading=>$bearing+$across,arc=>$scanner->arc_for_reach($distance+100),pitch=>$elevation+$up);
    $self->status->set(text=>'CUE',value=>$distance);
  }else{
    my$up=$self->{free}?(int($tick/10)%3-1)*35:0;
    $self->drive->set(throttle=>0.5,turn=>0.33,pitch_turn=>steer(0,$me->pitch));
    $scanner->set(heading=>$angles->normalize($scanner->last->{heading}+36),arc=>SEARCH_ARC,pitch=>$up);
    $self->status->set(text=>'SEARCH');
  }
  # A simple dodge: vary it, or predict the shooter.
  if($tick>=$self->{dodge_end} && grep {$_->{source} eq 'muzzle'} @{$self->sensors->cues}){
    $self->{dodges}++;$self->{dodge_end}=$tick+10;
  }
  if($tick<$self->{dodge_end}){
    # Off the line of fire: left, then right next time (turn is positive left, strafe right).
    my$left=$self->{dodges}%2==1;
    if($self->match->control eq 'forces'){$self->drive->set(throttle=>0.5);$self->drive->forces(strafe=>$left?-1:1);}
    else{$self->drive->set(throttle=>1,turn=>$left?1:-1);}
    $self->status->set(text=>$left?'dodge:left':'dodge:right',value=>$self->{dodges});
  }
  $self->drive->set(throttle=>$self->drive->get('throttle')*0.5)if$self->battery->charge<$self->battery->capacity/4;
  my$storm=$self->nav->storm;
  if($storm && $storm->margin($me->position)>-60){
    my$away=$angles->bearing_to($storm->{center},$me->position);
    $self->drive->set(throttle=>0.8,turn=>$angles->clamp($angles->angle_diff($away,$me->heading)/30));
  }
  my$zone=$self->nav->zone;
  if($zone && $zone->margin($me->position)<60){
    my$climb=$zone->{shape}eq'sphere'?$angles->elevation_to($me->position,$zone->{center}):0;
    $self->drive->set(throttle=>0.8,turn=>$angles->clamp($angles->angle_diff($angles->bearing_to($me->position,$zone->{center}),$me->heading)/30),pitch_turn=>$angles->clamp(($climb-$me->pitch)/30));
  }
}
package main;
Clunkyard::run(Starter->new);
