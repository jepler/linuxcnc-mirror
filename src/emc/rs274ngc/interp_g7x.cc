#include <list>
#include <tuple>
#include <fstream>
#include <iostream>
#include <deque>
#include <memory>
#include <complex>

#include "interp_internal.hh"

constexpr std::complex<double> I(0,1);

template <class T>
std::string to_string(T &d)
{
    std::ostringstream out;
    out << d;
    return out.str();
}

////////////////////////////////////////////////////////////////////////////////
class motion_base {
public:
    virtual ~motion_base() = default;
    virtual void straight_move(std::complex<double> end)=0;
    virtual void straight_rapid(std::complex<double> end)=0;
    virtual void circular_move(bool ccw,std::complex<double> center,
	std::complex<double> end)=0;
};

class motion_null:public motion_base {
public:
    void straight_move(std::complex<double> /*end*/) override  {}
    void straight_rapid(std::complex<double> /*end*/)  override {}
    void circular_move(bool /*ccw*/,std::complex<double> /*center*/,
	std::complex<double> /*end*/)  override {}
};

////////////////////////////////////////////////////////////////////////////////
class round_segment;
class straight_segment;

static constexpr double tolerance=1e-6;

class segment {
protected:
    std::complex<double> start, end;
    double finish{};
public:
    segment(double sz,double sx,double ez,double ex):start(sz,sx),end(ez,ex) {}
    segment(std::complex<double> s, std::complex<double> e):start(s),end(e) {}
    virtual ~segment() = default;
    typedef std::deque<double> intersections_t;
    virtual void intersection_z(double x, intersections_t &is)=0;
    virtual bool climb(std::complex<double>&, motion_base*)=0;
    virtual bool dive(std::complex<double>&,double, motion_base*,bool)=0;
    virtual void climb_only(std::complex<double>&, motion_base*)=0;
    virtual void draw(motion_base*)=0;
    virtual void offset(double)=0;
    virtual void intersect(segment*)=0;
    virtual void intersect_end(round_segment *p)=0;
    virtual void intersect_end(straight_segment *p)=0;
    virtual std::unique_ptr<segment> dup()=0;
    virtual double radius()=0;
    virtual bool monotonic() { return real(end-start)<=1e-3; }
    virtual void do_finish(segment * /*prev*/, segment * /*next*/) {}
    std::complex<double> &sp() { return start; }
    std::complex<double> &ep() { return end; }

    virtual void flip_imag() { start=conj(start); end=conj(end); }
    virtual void flip_real() { start=-conj(start); end=-conj(end); }
    virtual void rotate() {
	start=-start*I;
	end=-end*I;
    }
    virtual void move(std::complex<double> d) { start+=d; end+=d; }
    friend class round_segment;

protected:
    std::complex<double> corner_finish(segment *prev, segment *next) {
	auto p=prev->dup();
	auto n=next->dup();

	p->offset(finish);
	n->offset(finish);
	if(real(p->end-n->start)>1e-2) {
	    finish=-finish;
	    p=prev->dup();
	    n=next->dup();
	    p->offset(finish);
	    n->offset(finish);
	}
	if(real(p->end-n->start)>1e-2)
	    throw(std::string("Corner finish failed at ") + to_string(prev->end));
	p->intersect(n.get());
	std::complex<double> center=(p->end+n->start)/2.0;
	p->offset(-finish);
	n->offset(-finish);
	start=prev->end=p->end;
	end=next->start=n->start;
	return center;
    }
};

class straight_segment:public segment {
public:
    straight_segment(double sx, double sz,double ex, double ez):
	segment(sz,sx,ez,ex) {}
    straight_segment(std::complex<double> s,std::complex<double> e):
	segment(s,e) {}
    void intersection_z(double x, intersections_t &is) override;
    bool climb(std::complex<double>&,motion_base*) override;
    bool dive(std::complex<double>&,double, motion_base*,bool) override;
    void climb_only(std::complex<double>&,motion_base*) override;
    void draw(motion_base *out) override { out->straight_move(end); }
    void offset(double distance) override {
	std::complex<double> d=I*distance*(start-end)/abs(start-end);
	start+=d;
	end+=d;
    }
    void intersect(segment *p) override;
    void intersect_end(round_segment *p) override;
    void intersect_end(straight_segment *p) override;
    std::unique_ptr<segment> dup() override {
	return std::make_unique<straight_segment>(*this);
    }
    double radius() override { return abs(start-end); }
};

void straight_segment::intersection_z(double x, intersections_t &is)
{
    if(std::min(start.imag(),end.imag())>x+tolerance
	|| std::max(start.imag(),end.imag())<x-tolerance
    )
	return;
    if(std::abs(imag(start-end))<tolerance) {
	is.push_back(start.real());
	is.push_back(end.real());
    } else
	is.push_back((x-start.imag())
	    /(end.imag()-start.imag())*(end.real()-start.real())+start.real());
}

bool straight_segment::climb(std::complex<double> &location,
    motion_base *output
) {
    if(end.imag()+tolerance<start.imag())
	return 1; // not climbing
    if(abs(location-start)>tolerance)
	throw(std::string("How did we get here?"));
    output->straight_move(end);
    location=end;
    return 0;
}

void straight_segment::climb_only(std::complex<double> &location,
    motion_base *output
) {
    if(end.imag()<start.imag())
	return; // not climbing
    intersections_t is;
    intersection_z(location.imag(),is);
    if(is.empty())
	return;

    location.real(is.front());
    output->straight_move(location);
    output->straight_move(end);
    location=end;
    return;
}

bool straight_segment::dive(std::complex<double> &location,
    double x,
    motion_base *output, bool fast)
{
    if(start.imag()<=end.imag()+tolerance) 	// Not a diving segment
	return 1;
    if(x<end.imag()) {
	if(fast)
	    output->straight_rapid(end);
	else
	    output->straight_move(end);
	location=end;
	return 0;
    }
    intersections_t is;
    intersection_z(x,is);
    if(is.empty())
	throw(std::string("x too large in straight dive"));
    std::complex<double> ep(is.front(),x);
    if(fast)
	output->straight_rapid(ep);
    else
	output->straight_move(ep);
    location=ep;
    return 0;
}

class round_segment:public segment {
protected:
    int ccw;
    std::complex<double> center;
public:
    round_segment(bool c, double sx, double sz,
	double cx, double cz, double ex, double ez):
	segment(sz,sx,ez,ex), ccw(c), center(cz,cx)
    {
    }
    round_segment(bool cc, std::complex<double> s,
	std::complex<double> c, std::complex<double> e):
	segment(s,e), ccw(cc), center(c)
    {
    }
    void intersection_z(double x,intersections_t &is) override;
    bool climb(std::complex<double>&,motion_base*) override;
    bool dive(std::complex<double>&,double,motion_base*,bool) override;
    void climb_only(std::complex<double>&,motion_base*) override;
    void draw(motion_base *out) override { out->circular_move(ccw,center,end);}
    void offset(double distance) override {
	double factor=(abs(start-center)+(ccw? 1:-1)*distance)
	    /abs(start-center);
	start=factor*(start-center)+center;
	end=factor*(end-center)+center;
    }
    void intersect(segment *p) override;
    void intersect_end(round_segment *p) override;
    void intersect_end(straight_segment *p) override;
    std::complex<double> cp() { return center; }
    std::unique_ptr<segment> dup() override {
	return std::make_unique<round_segment>(*this);
    }
    double radius() override { return std::min(abs(start-end),abs(start-center)); }
    void flip_imag() override { ccw=!ccw; start=conj(start); end=conj(end);
	center=conj(center); }
    void flip_real() override { ccw=!ccw; start=-conj(start);
	end=-conj(end); center=-conj(center); }
    void rotate() override {
	start=-start*I;
	end=-end*I;
	center=-center*I;
    }
    bool monotonic() override {
	if(finish!=0)
	    return true;
	double entry=imag(start-center);
	double exit=imag(end-center);
	double dz=real(end-start);
	if(ccw)
	    return entry>=-1e-3 && exit>=-1e-3 && dz<=-1e-3;
	else
	    return entry<=1e-3 && exit<=1e-3 && dz<=-1e-3;
    }
    void move(std::complex<double> d) override { start+=d; center+=d; end+=d; }
private:
    bool on_segment(std::complex<double> p);
    friend class straight_segment;
};


inline bool round_segment::on_segment(std::complex<double> p)
{
    if(ccw)
	return imag(conj(start-center)*(p-center))>=-tolerance
	    && imag(conj(end-center)*(p-center))<=tolerance;
    else
	return imag(conj(start-center)*(p-center))<=tolerance
	    && imag(conj(end-center)*(p-center))>=-tolerance;
}

void round_segment::intersection_z(double x, intersections_t &is)
{
    std::complex<double> r=start-center;
    double s=-(x-abs(r)-center.imag())*(x+abs(r)-center.imag());
    if(s<-tolerance)
	return;
    if(s<0)
	s=0;
    s=sqrt(s);
    std::complex<double> p1(real(center+s),x);
    if(on_segment(p1))
	is.push_back(real(p1));
    std::complex<double> p2(real(center-s),x);
    if(on_segment(p2))
	is.push_back(real(p2));
}

bool round_segment::climb(std::complex<double> &location,
    motion_base *output
) {
    if(!ccw) { // G2
	if(location.real()>center.real())
	    return 1;
	if(abs(location-end)>1e-3)
	    output->circular_move(ccw,center,end);
	location=end;
	return 0;
    } else {
	if(location.real()<center.real())
	    return 1;
	std::complex<double> ep=end;
	if(end.real()<center.real()) {
	    ep.real(center.real());
	    ep.imag(center.imag()+abs(start-center));
	}
	if(abs(location-ep)>1e-3)
	    output->circular_move(ccw,center,ep);
	location=ep;
	return 1;
    }
}

bool round_segment::dive(std::complex<double> &location,
    double x, motion_base *output, bool
) {
    if(abs(location-end)<tolerance || real(location-end)<=tolerance) {
	location=end;
	return 0;
    }
    intersections_t is;
    intersection_z(x,is);
    if(is.empty())
	intersection_z(x-tolerance,is);

    if(ccw) { // G3
	if(location.real()-tolerance>center.real())
	    return 1;
	if(is.empty() || is.back()>real(center)) {
	    output->circular_move(ccw,center,end);
	    location=end;
	    return 0;
	} else {
	    std::complex<double> ep(is.back(),x);
	    output->circular_move(ccw,center,ep);
	    location=ep;
	    return 0;
	}
    } else {
	if(location.real()<=center.real())
	    return 1; // we're already climbing
	if(is.empty()) {
	    // Curve not hit, move to the end
	    output->circular_move(ccw,center,end);
	    location=end;
	    return 0;
	} else if(is.front()<real(center)) {
	    // also is.size()==1
	    // also abs(location-start)<tolerance
	    // curve hit at the back side only, let pocket decrement x
	    return 0;
	} else {
	    // Arc cut twice, move to the front intersection
	    std::complex<double> ep(is.front(),x);
	    output->circular_move(ccw,center,ep);
	    location=ep;
	    return 0;
	}
    }
}

void round_segment::climb_only(std::complex<double> &location,
    motion_base *output
) {
    intersections_t is;
    intersection_z(location.imag(),is);
    if(is.empty())
	return;
    if(!ccw) { // G2
	if(is.back()>center.real())
	    return;
	location.real(is.back());
	output->straight_move(location);
	if(abs(location-end)>1e-3)
	    output->circular_move(ccw,center,end);
	location=end;
	return;
    } else {
	if(is.front()<center.real())
	    return;
	location.real(is.front());
	output->straight_move(location);
	std::complex<double> ep=end;
	if(end.real()<center.real()) {
	    ep.real(center.real());
	    ep.imag(center.imag()+abs(start-center));
	}
	if(abs(location-ep)>1e-3)
	    output->circular_move(ccw,center,ep);
	location=ep;
	return;
    }
}


////////////////////////////////////////////////////////////////////////////////
void straight_segment::intersect(segment *p)
{
    p->intersect_end(this);
}

void round_segment::intersect(segment *p)
{
    p->intersect_end(this);
}

void straight_segment::intersect_end(straight_segment *p)
{
    // correct end of p and start of this
    auto rot=conj(start-end)/abs(start-end);
    auto ps=(p->start-end)*rot;
    auto pe=(p->end-end)*rot;
    if(imag(ps-pe)==0) {
	throw(std::string("Cannot intersect parallel lines"));
    }
    auto f=imag(ps)/imag(ps-pe);
    auto is=(ps+f*(pe-ps))/rot+end;
    start=p->end=is;
}

void straight_segment::intersect_end(round_segment *p)
{
    // correct end of p and start of this
    // (arc followed by a straight
    if(abs(start-p->end)<tolerance)
	return;

    auto rot=conj(start-end)/abs(start-end);
    auto pe=(p->end-end)*rot;
    auto pc=(p->center-end)*rot;

    double b=norm(pc-pe)-imag(pc)*imag(pc);
    if(b<0) {
	b=0;
    } else
	b=sqrt(b);

    auto s1=(real(pc)+b)/rot+end;
    auto s2=(real(pc)-b)/rot+end;

    if(abs(start-s1)<abs(start-s2))
	start=p->end=s1;
    else
	start=p->end=s2;
}

void round_segment::intersect_end(straight_segment *p)
{
    // correct end of p and start of this
    // (straight followed by an arc)
    if(abs(start-p->end)<tolerance)
	return;

    auto rot=conj(p->start-p->end)/abs(p->start-p->end);
    auto pe=(end-p->end)*rot;
    auto pc=(center-p->end)*rot;

    double b=norm(pc-pe)-imag(pc)*imag(pc);
    if(b<0) {
	b=0;
    } else
	b=sqrt(b);

    auto s1=(real(pc)+b)/rot+p->end;
    auto s2=(real(pc)-b)/rot+p->end;

    if(abs(start-s1)<abs(start-s2))
	start=p->end=s1;
    else
	start=p->end=s2;
}

void round_segment::intersect_end(round_segment *p)
{
    // correct end of p and start of this
    auto a=abs(start-center);
    auto b=abs(p->start-p->center);
    auto c=abs(center-p->center);
    auto cosB=(c*c+a*a-b*b)/2.0/a/c;
    double cosB2=cosB*cosB;
    if(cosB2>1)
	cosB2=1;
    std::complex<double> rot(cosB,sqrt(1-cosB2));
    auto is=rot*(p->center-center)/c*a+center;
    p->end=start=is;
}

////////////////////////////////////////////////////////////////////////////////
// Corner finishes

class fillet_segment:public round_segment {
public:
    fillet_segment(double d,std::complex<double> l):round_segment(0,l,l,l) {
	finish=d;
    }

    void do_finish(segment *prev, segment *next) override {
	center=corner_finish(prev,next);
	ccw=imag(start-center)>0 || imag(end-center)>0;
	finish=0;
    }
};

class chamfer_segment:public straight_segment {
public:
    chamfer_segment(double d,std::complex<double> l):straight_segment(l,l) {
	finish=d;
    }

    void do_finish(segment *prev, segment *next) override {
	corner_finish(prev,next);
    }
};


////////////////////////////////////////////////////////////////////////////////

template <int swap>
class swapped_motion:public motion_base {
    motion_base *orig;
public:
    swapped_motion(motion_base *motion):orig(motion) {
    }
    void straight_move(std::complex<double> end) override {
	switch(swap) {
	case 0: orig->straight_move(end); break;
	case 1: orig->straight_move(-conj(end)); break;
	case 2: orig->straight_move(conj(end)); break;
	case 3: orig->straight_move(-end); break;
	case 4: orig->straight_move(I*end); break;
	case 5: orig->straight_move(conj(I*end)); break;
	case 6: orig->straight_move(-conj(I*end)); break;
	case 7: orig->straight_move(-I*end); break;
	}
    }
    void straight_rapid(std::complex<double> end) override {
	switch(swap) {
	case 0: orig->straight_rapid(end); break;
	case 1: orig->straight_rapid(-conj(end)); break;
	case 2: orig->straight_rapid(conj(end)); break;
	case 3: orig->straight_rapid(-end); break;
	case 4: orig->straight_rapid(I*end); break;
	case 5: orig->straight_rapid(conj(I*end)); break;
	case 6: orig->straight_rapid(-conj(I*end)); break;
	case 7: orig->straight_rapid(-I*end); break;
	}
    }
    void circular_move(bool ccw,std::complex<double> center,
	std::complex<double> end) override
    {
	switch(swap) {
	case 0: orig->circular_move(ccw,center,end); break;
	case 1: orig->circular_move(!ccw,-conj(center),-conj(end)); break;
	case 2: orig->circular_move(!ccw,conj(center),conj(end)); break;
	case 3: orig->circular_move(ccw,-center,-end); break;
	case 4: orig->circular_move(ccw,I*center,I*end); break;
	case 5: orig->circular_move(!ccw,conj(I*center),conj(I*end)); break;
	case 6: orig->circular_move(!ccw,-conj(I*center),-conj(I*end)); break;
	case 7: orig->circular_move(ccw,-I*center,-I*end); break;
	}
    }
};

////////////////////////////////////////////////////////////////////////////////
class g7x:public  std::list<std::unique_ptr<segment>> {
    double delta{0.5};
    std::complex<double> escape{0.3, 0.3};
    int flip_state{};
    std::deque<std::complex<double>> pocket_starts;
private:
    void pocket(int cycle, std::complex<double> location, iterator p,
	motion_base *out);
    void add_distance(double distance);

    /* Rotate profile by 90 degrees */
    void rotate() {
	for(auto &p : *this)
	    p->rotate();
	flip_state^=4;
    }

    /* Change the direction of the profile to have Z and X decreasing */
    void swap() {
	double dir_x=imag(front()->ep()-back()->ep());
	double dir_z=real(front()->ep()-back()->ep());
	if(dir_x>0) {
	    for(auto &p : *this)
		p->flip_imag();
	    flip_state^=2;
	}
	if(dir_z<0) {
	    for(auto &p : *this)
		p->flip_real();
	    flip_state^=1;
	}
    }

    std::unique_ptr<motion_base> motion(motion_base *out) {
	switch(flip_state) {
	case 0: return std::make_unique<swapped_motion<0>>(out);
	case 1: return std::make_unique<swapped_motion<1>>(out);
	case 2: return std::make_unique<swapped_motion<2>>(out);
	case 3: return std::make_unique<swapped_motion<3>>(out);
	case 4: return std::make_unique<swapped_motion<4>>(out);
	case 5: return std::make_unique<swapped_motion<5>>(out);
	case 6: return std::make_unique<swapped_motion<6>>(out);
	case 7: return std::make_unique<swapped_motion<7>>(out);
	}
	throw(std::string("This can't happen"));
    }

    void monotonic() {
	if(real(front()->ep()-front()->sp())>0) {
	    front()->sp().real(real(front()->ep()));
	}
	for(auto &p : *this) {
	    if(!p->monotonic())
		throw(std::string("Not monotonic"));
	}
    }

    void do_finish() {
	for(auto h=++begin(); h!=--end(); ) {
	    auto p(h); --p;
	    (*h)->do_finish((*p).get(),(*(++h)).get());
	}
	for(auto p=begin(); p!=end(); p++)
	    if((*p)->radius()<1e-3)
		erase(p--);
    }

    bool should_rotate_paths() {
	g7x paths(*this);
	paths.pop_front();
	paths.swap();
	for(auto &path : paths) {
	    if(!path->monotonic())
		return true;
	}
	return false;
    }

public:
    g7x() = default;
    g7x(g7x const &other) : std::list<std::unique_ptr<segment>>() {
	delta=other.delta;
	escape=other.escape;
	flip_state=other.flip_state;
	for(const auto &p : other)
	    emplace_back(p->dup());
    }

    /*
	x,z	from where the distance is computed, also a rapid to this
		location between each pass.
	d	Starting distance
	e	Ending distance
	p	Number of passes to go from d to e
    */
    void do_g70(motion_base *out, double x, double z, double d, double e,
	int p, CUTTER_COMP *cutter_comp_side
    )
    {
	front()->sp()=std::complex<double>(z,x);

	if(should_rotate_paths())
	    rotate();
	swap();
	do_finish();
	monotonic();
	auto swapped_out=motion(out);

	for(int pass=p; pass>0; pass--) {
	    double distance=(pass-1)*(d-e)/p+e;
	    g7x paths(*this);
	    paths.add_distance(distance);

	    auto comp=*cutter_comp_side;
	    *cutter_comp_side=CUTTER_COMP::OFF;
	    swapped_out->straight_rapid(paths.front()->sp());
	    *cutter_comp_side=comp;
	    swapped_out->straight_rapid(paths.front()->ep());
	    paths.pop_front();
	    for(const auto &path : paths)
		path->draw(swapped_out.get());
	}
    }

    /*
	cycle	0 Complete cut including pockets.
		1 Do not cut any pockets.
		2 Only cut after first pocket (pick up where 1 stopped)
	x,z	From where the cutting is done.
	d	Final distance to profile
	i	Increment of cutting (depth of cut)
	r	Distance to retract
    */
    void do_g71(motion_base *out, int cycle, double x, double z,
	double u, double w,
	double d, double i, double r, bool do_rotate=false
    ) {
	front()->sp()=std::complex<double>(z,x);
	if(do_rotate)
	    rotate();
	swap();
	do_finish();
	monotonic();
	add_distance(d);
	std::complex<double> displacement(w,u);
	for(auto &p : *this)
	    p->move(displacement);
	auto swapped_out=motion(out);

	delta=std::max(i,2*tolerance);
	escape=r*std::complex<double>{1,1};

	swapped_out->straight_rapid(front()->sp());
	if(imag(back()->ep())<imag(front()->sp())) {
	    auto ep=back()->ep();
	    ep.imag(imag(front()->sp()));
	    emplace_back(std::make_unique<straight_segment>(
		back()->ep(),ep
	    ));
	}
	pocket_starts.push_back(front()->sp());
	pocket(cycle,front()->sp(), begin(), swapped_out.get());
    }

    void do_g72(motion_base *out, int cycle, double x, double z,
	double u, double w,
	double d, double i, double r
    ) {
	do_g71(out, cycle, x, z, u, w, d, i, r, true);
    }
};


void g7x::pocket(int cycle, std::complex<double> location, iterator p,
    motion_base *out
) {
    double x=imag(pocket_starts.back());

    if(cycle==2) {
	// This skips the initial roughing pass
	for(; p!=end(); p++) {
	    if((*p)->dive(location,-1e9,out,p==begin()))
		break;
	}
	cycle=3;
    }

    while(p!=end()) {
	while(x-tolerance>imag(location))
	    x-=delta;
	if((*p)->dive(location,x,out,p==begin())) {
	    if(cycle==1) {
		/* After the initial roughing pass, move along the final
		   contour and we're done
		*/
		for(; p!=end(); p++)
		    (*p)->climb_only(location,out);
	    } else {
		/* Move along the final contour until a pocket is found,
		   then start cutting that pocket
		*/
		for(; p!=end(); p++) {
		    if((*p)->climb(location,out)) {
			if(cycle==3) {
			    if(imag(location)>imag(pocket_starts.back())
				&& pocket_starts.size()>1
			    ) {
				pocket_starts.pop_back();
			    }
			    if(pocket_starts.size()==1) {
				pocket_starts.push_back(location);
			    }
			}
			pocket(cycle, location,p,out);
			return;
		    }
		}
	    }
	    return;
	}

        // We have added our move commands for the last path, we end the
        // recursive function before calculating and adding intersections.
	if (p==std::prev(end()))
	    return;

	if(std::abs(imag(location)-x)>tolerance) {
	    /* Our x coordinate is beyond the current segment, move onto
	       the next
	    */
	    p++;
	    continue;
	}

	for(auto ip=p; ip!=end(); ip++) {
	    segment::intersections_t is;
	    (*ip)->intersection_z(location.imag(),is);
	    if(is.empty())
		continue;

	    /* We can hit the diving curve, if its a circle */
	    double destination_z=ip!=p? is.front():is.back();
	    double distance=std::abs(destination_z-real(location));
	    if(destination_z-tolerance>real(location))
		continue; // Hitting the diving curve at the starting point only
	    if(distance<tolerance) {
		if(p==ip)
		    // Hitting the diving curve at the starting point.
		    continue;
		else if(abs(location-(*ip)->sp())<tolerance) {
		    // Another curve, at the entry point, move to that curve
		    p=ip;
		    break;
		} else {
		    // Just a zero length segment, consider it done
		    x-=delta;
		    break;
		}
	    }

	    std::complex<double> dest=location;
	    dest.real(destination_z);
	    out->straight_move(dest);
	    // If the travelled distance is too small for the entire escape
	    double escape_scale=std::min(1.0,distance/2/real(escape));
	    dest+=escape_scale*escape;
	    out->straight_rapid(dest);
	    dest=location-conj(escape_scale*escape);
	    out->straight_rapid(dest);
	    if(p==begin())
		out->straight_rapid(location);
	    else
		out->straight_move(location);
	    x-=delta;
	    break;
	}
    }
}

void g7x::add_distance(double distance) {
    auto v1=front()->ep()-front()->sp();
    auto v2=back()->sp()-front()->sp();
    auto angle=v1/v2;
    if(imag(angle)<0)
	distance=-distance;
    auto of(std::move(front()));
    pop_front();
    double current_distance=0;
    while(current_distance!=distance) {
	double max_distance=1e9;
	for(auto &p : *this)
	    max_distance=std::min(max_distance,p->radius()/2);
	max_distance=std::min(max_distance,std::abs(distance-current_distance));
	if(distance<0)
	    max_distance=-max_distance;

	for(auto p=begin(); p!=end(); p++) {
	    (*p)->offset(max_distance);
	    if((*p)->radius()<1e-3)
		erase(p--);
	}

	for(auto p=begin(); p!=--end(); p++) {
	    auto n=p; ++n;
	    if(real((*p)->ep()-(*n)->sp())>1e-2) {
		// insert connecting arc
		auto s((*p)->dup());
		auto e((*n)->dup());
		s->offset(-max_distance);
		e->offset(-max_distance);
		auto center=(s->ep()+e->sp())/2.0;
		emplace(n, std::make_unique<round_segment>(
		    distance>0,(*p)->ep(),center,(*n)->sp()));
		p++;
	    }
	}

	for(auto p=begin(); p!=--end(); p++) {
	    if(!(*p)->monotonic()) {
		std::cout << "Oops " << (*p)->sp()-(*p)->ep() << std::endl;
		auto pp=p; --pp;
		if((*p)->radius()<(*pp)->radius())
		    erase(p);
		else
		    erase(pp);
		p=begin();
	    }
	    auto n=p; ++n;
	    if((*p)->radius()<1e-3) {
		erase(p);
		p=begin();
	    } else
		(*p)->intersect(n->get());
	}
	current_distance+=max_distance;
    }
    for(auto p=begin(); p!=--end(); p++) {
	auto n=p; ++n;
	auto mid=((*p)->ep()+(*n)->sp())/2.0;
	(*p)->ep()=(*n)->sp()=mid;
    }

    of->ep()=front()->sp();
    push_front(std::move(of));
}

#ifndef IGNORE_LINUXCNC
////////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <string>
#include "rs274ngc.hh"
#include "rs274ngc_return.hh"
#include "rs274ngc_interp.hh"
#include "interp_internal.hh"
#include "units.h"

class motion_machine:public motion_base {
    Interp *interp;
    setup_pointer settings;
    block_pointer block;
public:
    motion_machine(Interp *i, setup_pointer s, block_pointer b):
	interp(i), settings(s), block(b) { }

    void straight_move(std::complex<double> end) override {
	block->x_flag=1;
	block->x_number=imag(end);
	block->z_flag=1;
	block->z_number=real(end);
	int r=interp->convert_straight(G_1, block, settings);
	if(r!=INTERP_OK)
	    throw(r);
    }

    void straight_rapid(std::complex<double> end) override  {
	block->x_flag=1;
	block->x_number=imag(end);
	block->z_flag=1;
	block->z_number=real(end);
	int r=interp->convert_straight(G_0, block, settings);
	if(r!=INTERP_OK)
	    throw(r);
    }
    void circular_move(bool ccw,std::complex<double> center,
	std::complex<double> end
    ) override {
	block->x_flag=1;
	block->x_number=imag(end);
	block->z_flag=1;
	block->z_number=real(end);
	block->i_flag=1;
	block->i_number=imag(center);
	block->k_flag=1;
	block->k_number=real(center);

	if(!equal(settings->current_z,block->z_number)
	    || !equal(settings->current_x,block->x_number)
	) {
	    int r=interp->convert_arc(ccw? G_3:G_2, block, settings);
	    if(r!=INTERP_OK)
		throw(r);
	}
    }
};

class switch_settings {
    Interp *interp;
    setup_pointer settings;
    DISTANCE_MODE saved_distance_mode, saved_ijk_distance_mode;
    read_function_pointer read_a, read_c, read_u, read_w;
public:
    switch_settings(Interp *i,setup_pointer s):interp(i), settings(s)
    {
	saved_distance_mode=settings->distance_mode;
	settings->distance_mode=DISTANCE_MODE::ABSOLUTE;
	saved_ijk_distance_mode=settings->ijk_distance_mode;
	settings->ijk_distance_mode=DISTANCE_MODE::ABSOLUTE;
	read_a=interp->_readers[(int)'a'];
	read_c=interp->_readers[(int)'c'];
	read_u=interp->_readers[(int)'u'];
	read_w=interp->_readers[(int)'w'];
	interp->_readers[(int)'a']=Interp::default_readers[(int)'a'];
	interp->_readers[(int)'c']=Interp::default_readers[(int)'c'];
	interp->_readers[(int)'u']=Interp::default_readers[(int)'u'];
	interp->_readers[(int)'w']=Interp::default_readers[(int)'w'];
    }
    ~switch_settings() {
	settings->distance_mode=saved_distance_mode;
	settings->ijk_distance_mode=saved_ijk_distance_mode;
	interp->_readers[(int)'a']=read_a;
	interp->_readers[(int)'c']=read_c;
	interp->_readers[(int)'u']=read_u;
	interp->_readers[(int)'w']=read_w;
    }
    DISTANCE_MODE ijk_distance_mode() { return saved_ijk_distance_mode; }
    DISTANCE_MODE distance_mode() { return saved_distance_mode; }
};

int Interp::convert_g7x(int /*mode*/,
      block_pointer block,     //!< pointer to a block of RS274 instructions
      setup_pointer settings)  //!< pointer to machine settings
{

    if(!block->q_flag)
    	ERS("G7x.x  requires a Q word");

    int cycle=block->g_modes[GM_MOTION];
    int subcycle=cycle%10;
    cycle/=10;

    if(settings->cutter_comp_side != CUTTER_COMP::OFF && cycle!=70)
	ERS("G%d.%d cannot be used with cutter compensation enabled",
	    cycle, subcycle);
    if(settings->plane!=CANON_PLANE::XZ)
	ERS("G%d.%d can only be used in XZ plane (G18)",
	    cycle, subcycle);

    switch_settings old(this,settings);

    auto original_block=*block;

    double x=settings->current_x;
    double z=settings->current_z;
    double start_x=x;
    double start_z=z;
    if(old.distance_mode()==DISTANCE_MODE::INCREMENTAL) {
	if(block->x_flag)
	    x+=block->x_number;
	if(block->z_flag)
	    z+=block->z_number;
    } else {
	if(block->x_flag)
	    x=block->x_number;
	if(block->z_flag)
	    z=block->z_number;
    }
    original_block.x_number=x;
    original_block.z_number=z;

    auto cutter_comp=settings->cutter_comp_side;
    settings->cutter_comp_side=CUTTER_COMP::OFF;
    int error=convert_straight(G_0, block, settings);
    settings->cutter_comp_side=cutter_comp;
    if(error!=INTERP_OK)
	return error;

    g7x path;
    std::complex<double> start(z,x);

    auto exit_call_level=settings->call_level;
    CHP(read((std::string("O")+std::to_string(static_cast<int>(block->q_number))+" CALL").c_str()));
    for(;;) {
	if(block->o_name!=0)
	    CHP(convert_control_functions(block, settings));
	if(settings->call_level==exit_call_level)
	    break;

	for(int n=0; n<settings->parameter_occurrence; n++)
	    settings->parameters[settings->parameter_numbers[n]]=
		settings->parameter_values[n];

	for(int n=0; n<settings->named_parameter_occurrence; n++)
	    CHP(store_named_param(&_setup, settings->named_parameters[n],
		settings->named_parameter_values[n]
	    ));
	settings->named_parameter_occurrence = 0;

	std::complex<double> end(start);
	std::complex<double> center(0,0);

	if(old.distance_mode()==DISTANCE_MODE::INCREMENTAL)
	    end=0;

	if(block->u_flag) {
	    if(old.distance_mode()==DISTANCE_MODE::INCREMENTAL)
		ERS("G7x error: Cannot use U in incremental mode (G91)");
	    if(block->x_flag)
		ERS("G7x error: Cannot use U and X in the same block");
	    auto u=block->u_number;
	    if(settings->lathe_diameter_mode)
		u/=2;
	    end.imag(end.imag()+u);
	} else if(block->x_flag)
	    end.imag(block->x_number);

	if(block->w_flag) {
	    if(old.distance_mode()==DISTANCE_MODE::INCREMENTAL)
		ERS("G7x error: Cannot use W in incremental mode (G91)");
	    if(block->z_flag)
		ERS("G7x error: Cannot use W and Z in the same block");
	    end.real(end.real()+block->w_number);
	} else if(block->z_flag)
	    end.real(block->z_number);

	if(block->i_flag) center.imag(block->i_number);
	if(block->k_flag) center.real(block->k_number);

	if(old.distance_mode()==DISTANCE_MODE::INCREMENTAL)
	    end+=start;

	if(block->g_modes[GM_MOTION]!=-1)
	    settings->motion_mode=block->g_modes[GM_MOTION];
	if(start!=end) {
	    switch(settings->motion_mode) {
	    case 0:
	    case 10:
		path.emplace_back(std::make_unique<straight_segment>(
		    start, end
		));
		break;
	    case 20:
	    case 30:
		if(block->r_flag) {
		    if(block->i_flag || block->k_flag)
			ERS("G7X error: both R and I or K flag used for arc");
		    double r=block->r_number;
		    center=(start+end)/2.0;
		    auto d=I*sqrt((r*r-norm(end-start)/4)/norm(end-start))
			*(end-start);
		    if(settings->motion_mode==30)
			center+=d;
		    else
			center-=d;
		} else {
		    if(!block->i_flag && !block->k_flag)
			ERS("G7X error: either I or K must be present for arc");
		    if(old.ijk_distance_mode()==DISTANCE_MODE::INCREMENTAL)
			center+=start;
		}

        /* Verify if circular motion is valid */
        double tpx;
        double tpy;
        double tpz;
        double tAA_p;
        double tBB_p;
        double tCC_p;
        double tu_p;
        double tv_p;
        double tw_p;
        CHP(find_ends(block, settings, &tpx, &tpy, &tpz, &tAA_p, &tBB_p, &tCC_p, &tu_p, &tv_p, &tw_p));

        if(!block->r_flag){
            double center1, center2;
            int turn;
            double radius_tolerance = (settings->length_units == CANON_UNITS_INCHES) ? RADIUS_TOLERANCE_INCH : RADIUS_TOLERANCE_MM;
            double spiral_abs_tolerance = (settings->length_units == CANON_UNITS_INCHES) ? settings->center_arc_radius_tolerance_inch : settings->center_arc_radius_tolerance_mm;
            CHP(arc_data_ijk((settings->motion_mode==30)? G_3 : G_2, settings->plane, settings->current_z, settings->current_x, tpz, tpx,
                             (old.ijk_distance_mode() == DISTANCE_MODE::ABSOLUTE),
                             (block->k_flag)? block->k_number : 0.0, (block->i_flag)? block->i_number : 0.0, block->p_flag? round_to_int(block->p_number) : 1,
                             &center1, &center2, &turn, radius_tolerance, spiral_abs_tolerance, SPIRAL_RELATIVE_TOLERANCE));
        }
        /*************/

		path.emplace_back(std::make_unique<round_segment>(
		    settings->motion_mode==30, start, center, end
		));
		break;
	    }
	    if(settings->motion_mode==0 || settings->motion_mode==10
		|| settings->motion_mode==20 || settings->motion_mode==30
	    ) {
		if(block->a_flag && block->c_flag)
		    ERS("G7X error: Both A and C parameters on a corner");
		if(block->a_flag)
		    path.emplace_back(std::make_unique<fillet_segment>(
			block->a_number, end
		    ));
		if(block->c_flag)
		    path.emplace_back(std::make_unique<chamfer_segment>(
			block->c_number, end
		    ));
	    }

	    settings->current_x=imag(end);
	    settings->current_z=real(end);
	    start=end;
	}
	CHP(read());
    }
    if(path.size()<=1)
	return INTERP_OK;

    double d=0, e=0, i=1, r=0.5, u=0, w=0;
    int p = 1;
    if(original_block.d_flag) d=original_block.d_number_float;
    if(original_block.e_flag) e=original_block.e_number;
    if(original_block.i_flag) i=original_block.i_number;
    if(original_block.p_flag) p=static_cast<int>(original_block.p_number);
    if(original_block.r_flag) r=original_block.r_number;
    if(original_block.u_flag) u=original_block.u_number;
    if(original_block.w_flag) w=original_block.w_number;
    if(original_block.x_flag) x=original_block.x_number;
    if(original_block.z_flag) z=original_block.z_number;

    settings->current_x=start_x;
    settings->current_z=start_z;

    if(i<=0)
	ERS("G7X error: I must be greater than zero.");

    auto dfront=path.front()->ep()-path.front()->sp();
    auto dback=path.back()->ep()-path.back()->sp();
    motion_machine motion(this, settings, block);
    try {
	switch(cycle) {
	case 70:
	    path.do_g70(&motion,x,z,d,e,p,&settings->cutter_comp_side);
	    break;
	case 71:
	    if(std::abs(real(dback))>0 && imag(dfront)*(x-imag(start))<0) {
		std::complex<double> end{real(start),x};
		path.emplace_back(std::make_unique<straight_segment>(
		    start, end
		));
	    }
	    path.do_g71(&motion,subcycle,x,z,u,w,d,i,r);
	    break;
	case 72:
	    if(std::abs(imag(dback))>0 && real(dfront)*(z-real(start))<0) {
		std::complex<double> end{z,imag(start)};
		path.emplace_back(std::make_unique<straight_segment>(
		    start, end
		));
	    }
	    path.do_g72(&motion,subcycle,x,z,u,w,d,i,r);
	    break;
	}
    } catch(std::string &s) {
	ERS("G7X error: %s", s.c_str());
	return INTERP_ERROR;
    } catch(int i) {
	return i;
    }

    return INTERP_OK;
}
#endif
