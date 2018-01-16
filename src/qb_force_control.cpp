#include <qb_force_control.h>



qb_force::qb_force(){
    int br;
    std::string port;
    std::string aux;
    std::vector<int> ID_hand;

    node_ = new ros::NodeHandle("qb_interface_node_");
    node_->param<double>("/step_time", step_time_, 0.002);
    node_->param<std::string>("/port", port, "/dev/ttyUSB0");
    node_->param<int>("/baudrate", br, 2000000);
    node_->searchParam("/IDhands", aux);
    node_->getParam(aux, ID_hand);
    qb_comm_ = NULL;
    if (!open(port.c_str(), br)){
        qb_comm_ = NULL;

        cout << "[ERROR] USB " << port << " was not open correctly." << endl;
        return;
    }

    qbHand* tmp_hand;

    // Allocate object qbCube, one for each cube in cubeBuf

    for (int i = ID_hand.size(); i--;) {

        tmp_hand = new qbHand(qb_comm_, ID_hand[i]);

        // IF an error is find
        if (tmp_hand == NULL){
            cout << "[ERORR] Unable to allocate space for cube structure." << endl;
            return;
        }

        hand_chain_.push_back(tmp_hand);
    }

    // Activate
    if (!activate())
        cout << "[ERROR] Some qbDevice are not correctly activated." << endl;

    // Initialize publisher and subscriber

    if (!hand_chain_.empty()){
        hand_sub = node_->subscribe("/qb_class/hand_ref", 1, &qb_force::handRefCallback, this);
        hand_pub = node_->advertise<qb_interface::handPos>("/qb_class/hand_measurement", 1);
    }
}


void qb_force::handRefCallback(const qb_interface::handRef::ConstPtr& msg){
    pos_ref_ = msg->closure;
}

qb_force::~qb_force(){


    // Deactivate
    if (!deactivate())
        cout << "[ERROR] Some qbDevices was not correctly deactivated." << endl;

    // Close port
    close();

    qb_comm_ = NULL;

}


void qb_force::spin(){
    // 1/step_time is the rate in Hz
    ros::Rate loop_rate(1.0 / step_time_);
    while(ros::ok()) {
        spinOnce();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool qb_force::readMeasCurrent() {
    short int p_meas[3]={0,0,0};
    short int p_current[3]={0,0,0};
    bool ret;


    for(int i=0;i<hand_chain_.size();i++){
        ret=hand_chain_[i]->getMeas(p_meas) && hand_chain_[i]->getCurrents(p_current);

        ROS_DEBUG_STREAM("\np_meas:" << p_meas[0] << "."<<p_meas[1]<<"."<<p_meas[2]<<"\np_current:" << p_current[0] << "."<<p_current[1]<<"."<<p_current[2]<<"\n");

        if (p_meas[0]!=0) {
            pos_.clear();
            pos_.push_back((float) p_meas[0]);

        }
        if (p_current[0]!=0) {
            cur_.clear();
            cur_.push_back((float) p_current[0]);
        }
    }
    return ret;
}
void qb_force::spinOnce(){

    if (qb_comm_ == NULL){
        cout << "[ERROR] Connection error in spin() function." << endl;
        return;
    }

    // Read positions and Currents of all devices
    readMeasCurrent();
    qb_interface::handPos state;
    if (pos_.size()>0 && cur_.size()>0) {
        state.closure.push_back((float) pos_.at(0));
        state.closure.push_back((float) cur_.at(0));
        // Set Position of all devices
        float AAA;
        move(AAA);
        state.closure.push_back(AAA);
        hand_pub.publish(state);
    }
}

bool qb_force::open(const char* port, const int br) {

    // Check connection state
    if (qb_comm_ != NULL)
        return false;

    qb_comm_ = new comm_settings;

    // Call open communication function

    if (br >= 2000000)
        openRS485(qb_comm_, port, B2000000);
    else
        openRS485(qb_comm_, port, B460800);

    if (qb_comm_->file_handle == INVALID_HANDLE_VALUE)
        return false;

    // Set all hands communication

    for (int i = hand_chain_.size(); i--;)
        hand_chain_[i]->cube_comm_ = qb_comm_;

    return true;
}

//-----------------------------------------------------
//                                                close
//-----------------------------------------------------

/*
/ *****************************************************
/ Close connection of all cubes and hands
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qb_force::close() {

    // Check connection status

    if (qb_comm_ == NULL)
        return false;

    short int meas[2];
    meas[0]=00;
    meas[1]=00;
    hand_chain_[0]->setInputs(meas);
    // Call ccommunication function

    closeRS485(qb_comm_);

    delete qb_comm_;

    qb_comm_ = NULL;

    // Set all hands communication

    for (int i = hand_chain_.size(); i--;)
        hand_chain_[i]->cube_comm_ = qb_comm_;

    return true;
}

bool qb_force::activate() {

    bool status = true;

    // Check connection status
    if (qb_comm_ == NULL)
        return false;

    int err_count = 0;

    for (int i = hand_chain_.size(); i--;){

        while(!(hand_chain_[i]->activate()) && (++err_count < ERR_TIMEOUT));

        if (err_count == ERR_TIMEOUT)
            status = false;
        err_count = 0;
    }

    return status;
}

//-----------------------------------------------------
//                                           deactivate
//-----------------------------------------------------

/*
/ *****************************************************
/ Close connection of all cubes and hands
/ *****************************************************
/   parameters:
/   return:
/       true  on success
/       false on failure
/
*/

bool qb_force::deactivate() {

    bool status = true;

    // Check connection status
    if (qb_comm_ == NULL)
        return false;

    int err_count = 0;

    for (int i = hand_chain_.size(); i--;){

        while(!(hand_chain_[i]->deactivate()) && (++err_count < ERR_TIMEOUT));

        if (err_count == ERR_TIMEOUT)
            status = false;
        err_count = 0;
    }

    return status;
}

float qb_force::calc_current(float set_pos,float cur_pos,float stiffness, float speed, float damping,float deadband){
    //THIS IS WHERE WE CALCULATE THE CURRENT
    float error=(set_pos-cur_pos);
    if (fabs(error)>deadband)   return error*stiffness;
    else return 0;
}
void qb_force::move(float &debug_ref) {

    if (pos_ref_.size()>0) {
        node_->param<double>("/stiffness", stiffness_, 0.1);
        short int meas[2];
        float des_cur = calc_current(pos_ref_.at(0), pos_.at(0), stiffness_, speed_, damping_,0);
        meas[0] = (short) des_cur;
        meas[1] = (short) des_cur;
        hand_chain_[0]->setInputs(meas);
        debug_ref=des_cur;
    }
/*

    // Check if new position are received
    if ((p_1_.size() != 0) && (p_2_.size() != 0)){

        // Cubes
        if (flagCMD_type_ == EQ_PRESET){



            // Command cubes in Equilibrium Position and Preset
            for (int i = cube_chain_.size(); i--;)
                cube_chain_[i]->setPosAndPreset(p_1_[(cube_chain_.size() - 1) - i], p_2_[(cube_chain_.size() - 1) - i], meas_unit_);


        }else{

            // Command cubes in Pos1 and Pos2
            short int meas[2];
            float f_meas[2];

            for (int i = cube_chain_.size(); i--;){
                f_meas[0] = p_1_[(cube_chain_.size() - 1) - i];
                f_meas[1] = p_2_[(cube_chain_.size() - 1) - i];

                // Convert in TICK
                if (meas_unit_ == DEG){
                    f_meas[0] *= DEG_TICK_MULTIPLIER;
                    f_meas[1] *= DEG_TICK_MULTIPLIER;
                }
                else{
                    if (meas_unit_ == RAD){
                        f_meas[0] = (f_meas[0] / (M_PI / 180.0)) * DEG_TICK_MULTIPLIER;
                        f_meas[1] = (f_meas[1] / (M_PI / 180.0)) * DEG_TICK_MULTIPLIER;
                    }
                }

                meas[0] = (short int) f_meas[0];
                meas[1] = (short int) f_meas[1];

                cube_chain_[i]->setInputs(meas);
            }
        }
    }
    // Hands

    // Check if new positions are received
    if (pos_.size() != 0){

        if (flag_HCMD_type_ == PERC){

            // Command hand in percents

            for (int i = hand_chain_.size(); i--;)
                hand_chain_[i]->setPosPerc(pos_[(hand_chain_.size() - 1) - i]);

        }else{

            // Command hand in Pos1 and Pos2, TICK
            short int meas[2];

            for (int i = hand_chain_.size(); i--;){
                meas[0] = (short int) pos_[(hand_chain_.size() - 1) - i];
                meas[1] = (short int) pos_[(hand_chain_.size() - 1) - i];

                hand_chain_[i]->setInputs(meas);
            }
        }
    }
    */


}
