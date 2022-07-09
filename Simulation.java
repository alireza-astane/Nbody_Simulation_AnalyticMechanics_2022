package org.example;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import static java.lang.Math.*;

public class Simulation {
    int step = 1;
    int N = 30;
    double m = pow(10, -4);
    double k = -pow(10, -10);
    double side = 100;
    double initSide = 1;
    double initAverageSpeed = 0.1;
    double totalTime = 1;
    double timeStep = 0.0001;
    double[] k_data;
    double[] H_data;
    ArrayList<Obj> Objects = new ArrayList<>();
    // r data

    public Simulation() {
        for (int i = 0; i < N; i++) {
            Obj obj = new Obj();
            Objects.add(obj);
        }

    }

    public double getKData() {
        double res = 0;
        for (Obj obj : Objects) {
            res += 0.5 * m * (pow(obj.vel.get(0), 2) + pow(obj.vel.get(1), 2));
        }
        return res;

    }

    public double getHData() {
        double u = 0;
        for (int i = 1; i < N; i++) {
            for (int j = i + 1; j < N + 1; j++) {
                u += k * log(dist(i, j));
            }
        }
        return u + k_data[step - 1];

    }

    public Obj getObject(int i) {
        return Objects.get(i - 1);
    }

    public void next() {
        for (Obj obj : Objects) {
            ArrayList<Double> a1 = acceleration();
            obj.pos = obj.pos + obj.vel * timeStep + a1 * pow(timeStep, 2) / 2;   //wall?
            ArrayList<Double> a2 = acceleration();
            obj.vel = obj.vel + (a1 + a2) * timeStep / 2l;
        }

    }

    public void generate() {
        while (step < (int) (totalTime / timeStep)) {
            k_data[step - 1] = getKData();
            H_data[step - 1] = getHData();
            //add pos data
            next();
            step++;
        }

    }

    public ArrayList<Double> acceleration(int i) {
        for (int j = 1; i < N + 1; i++) {
            if (j != i) {
                acc +=
            }
            return -k / (2 * m) *


        }
    }

        public void wall () {
        }

        public double dist ( int i, int j){
            return sqrt(pow(getObject(i).pos.get(0) - getObject(j).pos.get(0), 2) + pow(getObject(i).pos.get(1) - getObject(j).pos.get(1), 2));
        }

        public void plotK () {
        }

        public void plotH () {
        }

        public double getT () {
        }

        public void animate () {
        }
        public class Obj {
            ArrayList<Double> pos;
            ArrayList<Double> vel;

            public Obj() {
                pos = new ArrayList<Double>(Arrays.asList(initSide * new Random().nextDouble(), initSide * new Random().nextDouble()));  //pos from -initSide to +initSide
                vel = new ArrayList<Double>(Arrays.asList(initAverageSpeed * new Random().nextDouble(), initAverageSpeed * new Random().nextDouble()));
            }  //check distribution

            public ArrayList<Double> getPos() {
                return pos;
            }

            public ArrayList<Double> getVel() {
                return vel;
            }

            public void setPos(ArrayList<Double> pos) {
                this.pos = pos;
            }

            public void setVel(ArrayList<Double> vel) {
                this.vel = vel;
            }

        }
    }
}
    
    
    
    
    
    
    
    
    
}
