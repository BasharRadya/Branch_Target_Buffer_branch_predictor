/* 046267 Computer Architecture - HW #1                                 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"





#include <stdio.h>
#include <cstdint>
#include <cmath>
#include <iostream>

#define targetSize 30
enum bimodal_state {
    STRONG_NOT_TAKEN,  // 0
    WEAK_NOT_TAKEN,    // 1
    WEAK_TAKEN,        // 2
    STRONG_TAKEN       // 3
};
enum share_state{
    NOT_SHARING,  // 0
    SHARING_LSB,    // 1
    SHARING_MID        // 2
};
class bimodal_SM{
    public:
        bimodal_state state;
        bimodal_SM()=default;
        void set_state(bimodal_state new_state){
            state = new_state;
        }
        void go_taken(){
            switch (state)
            {
            case STRONG_TAKEN:
                break;
            case WEAK_TAKEN:
                state = STRONG_TAKEN;
                break;
            case WEAK_NOT_TAKEN:
                state = WEAK_TAKEN;
                break;
            case STRONG_NOT_TAKEN:
                state = WEAK_NOT_TAKEN;
                break; 
            default:
                printf("error in bimodal_SM::go_not_taken\n");
                break;
            }
        }
        void go_not_taken(){
            switch (state)
            {
            case STRONG_TAKEN:
                state = WEAK_TAKEN;
                break;
            case WEAK_TAKEN:
                state = WEAK_NOT_TAKEN;
                break;
            case WEAK_NOT_TAKEN:
                state = STRONG_NOT_TAKEN;
                break;
            case STRONG_NOT_TAKEN:
                break; 
            default:
                printf("error in bimodal_SM::go_not_taken\n");
                break;
            }
        }

};
class counter_array{
    public:
        uint32_t array_size;
        uint32_t history_size_in_power;
        bimodal_SM* bimodal_sm_array;
        counter_array(uint32_t history_size_in_power, bimodal_state init_state){
            this->history_size_in_power = history_size_in_power;
            this->array_size = 1<<history_size_in_power;
            bimodal_sm_array = new bimodal_SM[array_size];
            for (uint32_t i = 0; i < array_size; i++)
            {
                bimodal_sm_array[i].set_state(init_state);
            }
        }

        bimodal_SM* find_bimodal_sm_sharing(uint32_t history, uint32_t pc,share_state sharing_state){
         
        uint32_t pc_new;
        switch(sharing_state){
            case SHARING_LSB:
            pc_new = pc>>2;
            break;
            case SHARING_MID:
            pc_new = pc>>16;
            break;
            default:
                 printf("error in counter_array::find_bimodal_sm_sharing\n");
                 break;
         }
         uint32_t index = (history^pc_new)  & ((1<<history_size_in_power)-1);
         return &bimodal_sm_array[index];
        }
        void reset_counter_array(bimodal_state init_state){
            for (uint32_t i = 0; i < array_size; i++)
            {
                bimodal_sm_array[i].set_state(init_state);
            }
        }
        bimodal_SM* find_bimodal_sm(uint32_t history){
         uint32_t index = history & ((1<<history_size_in_power)-1);
         return &bimodal_sm_array[index];
        }
};

class btb{
    public:
   struct btb_entry{
        uint32_t tag;
        bool valid;
        uint32_t* history;
        uint32_t target;
        counter_array* table_arr;
    };
    unsigned tag_size;
    unsigned history_size;
    unsigned btb_size;
    unsigned btb_size_in_power;
    bimodal_state btb_sm_init_state;
    bool isGlobalHist;
    bool isGlobalTable;
    share_state shared;
    btb_entry* btb_array;
    SIM_stats stats;
    void init_btb_array(){
        for (unsigned i = 0; i < btb_size; i++)
        {
            btb_array[i].valid = false;
            if(isGlobalHist){
                if(i!=0){
                    btb_array[i].history = btb_array[0].history;
                }else{                  
                    btb_array[i].history = new uint32_t(0);
                    *(btb_array[i].history) = 0;
                }
            }else{
                btb_array[i].history = new uint32_t(0);
            }
            btb_array[i].tag = 0;
            btb_array[i].target = 0;
            if(isGlobalTable){
                if(i!=0){
                    btb_array[i].table_arr = btb_array[0].table_arr;
                }else{
                    btb_array[i].table_arr = new counter_array(history_size,btb_sm_init_state);
                }
            }else{
                btb_array[i].table_arr = new counter_array(history_size,btb_sm_init_state);
                if(btb_array[i].table_arr == nullptr){
                    printf("null table_arr , faield to allocate\n");
                }
            }
        }
    } 
    btb(unsigned btb_size, unsigned history_size, unsigned tag_size,unsigned fmState ,bool isGlobalHist, bool isGlobalTable, int Shared){
        this->btb_size = btb_size;
        this->btb_size_in_power = std::log2(btb_size);
        this->history_size = history_size;
        this->tag_size = tag_size;
        this->btb_sm_init_state = (bimodal_state)fmState;

        this->isGlobalHist = isGlobalHist;
        this->isGlobalTable = isGlobalTable;
        this->shared = (share_state)Shared;
        btb_array = new btb_entry[btb_size];
        init_btb_array();
        stats={0,0,0};
    }

   btb_entry* find_btb_entry(uint32_t pc){
         uint32_t index = pc>>2 & ((1<<btb_size_in_power)-1);
         return &btb_array[index];
    }
    void change_history(uint32_t* history, bool taken){
        *history = *history << 1;
        *history = *history & ((1<<history_size)-1);
        *history = *history | (taken ? 1:0);
    }
    bool check_entry_tag_equal(btb_entry* entry, uint32_t pc){
        return (entry->tag == ((pc >> 2) & ((1<<tag_size)-1)));
    }
    void update_btb_entry(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
        stats.br_num++;
        if (taken){
                if (!(pred_dst == targetPc)){
                    stats.flush_num++ ;
                }
            }else {
                if (!(pred_dst == (pc + 4))){
                    stats.flush_num++ ;
                }
        }
        btb_entry* entry = find_btb_entry(pc);
        if(entry->valid == false){
            entry->valid = true;
            entry->tag = (pc >> 2) & ((1<<tag_size)-1);
            entry->target = targetPc;
            if(!isGlobalTable){
                entry->table_arr->reset_counter_array(btb_sm_init_state);
            }
            if(!isGlobalHist){
                *(entry->history)=0;
            }
            bimodal_SM* sm;
                if(shared == NOT_SHARING){
                    sm = entry->table_arr->find_bimodal_sm(*(entry->history));
                }else{
                    sm = entry->table_arr->find_bimodal_sm_sharing(*(entry->history),pc,shared);
                }
                            change_history(entry->history,taken);

                //taken is the correct decision that should be taken (the actual decision after calculating EXE) 
                if(taken){
                        sm->go_taken();
                }else{
                        sm->go_not_taken();
                }
                // we assume that we need to change the history in EXE (written in update discription)
        }else{
            if(check_entry_tag_equal(entry,pc)){
                bimodal_SM* sm;
                if(shared == NOT_SHARING){
                    sm = entry->table_arr->find_bimodal_sm(*(entry->history));
                }else{
                    sm = entry->table_arr->find_bimodal_sm_sharing(*(entry->history),pc,shared);
                }
                //taken is the correct decision that should be taken (the actual decision after calculating EXE) 
                if(taken){
                        sm->go_taken();
                }else{
                        sm->go_not_taken();
                }
                // we assume that we need to change the history in EXE (written in update discription)
                change_history(entry->history,taken);
            }else{
                entry->tag = (pc >> 2) & ((1<<tag_size)-1);
                entry->target = targetPc;
                if(!isGlobalTable){
                    entry->table_arr->reset_counter_array(btb_sm_init_state);
                }
                if(!isGlobalHist){
                    *(entry->history)=0;
                }
                bimodal_SM* sm;
                if(shared == NOT_SHARING){
                    sm = entry->table_arr->find_bimodal_sm(*(entry->history));
                }else{
                    sm = entry->table_arr->find_bimodal_sm_sharing(*(entry->history),pc,shared);
                }
                // we assume that we need to change the history in EXE (written in update discription)
                //taken is the correct decision that should be taken (the actual decision after calculating EXE) 
                if(taken){
                        sm->go_taken();
                }else{
                        sm->go_not_taken();
                }
                change_history(entry->history,taken);

            }
        }
    }
    bool predit(uint32_t pc, uint32_t* dst){
        btb_entry* entry = find_btb_entry(pc);
        if(entry->valid == false){
            *dst = pc + 4;
            return false;
        }else{
            if(check_entry_tag_equal(entry,pc)){
                bimodal_SM* sm;
                if(shared == NOT_SHARING){
                    sm = entry->table_arr->find_bimodal_sm(*(entry->history));
                }else{
                    sm = entry->table_arr->find_bimodal_sm_sharing(*(entry->history),pc,shared);
                }
                if(sm->state == STRONG_TAKEN || sm->state == WEAK_TAKEN){
                    *dst = entry->target;
                    return true;
                }else{
                    *dst = pc + 4;
                    return false;
                }
            }else{
                *dst = pc + 4;
                return false;
            }
        }
        printf("error in btb::predit\n");
        return false;
    }
    void calculate_predictor_hypothesis_size(){
        unsigned all_tags_and_valid_bit = btb_size * (tag_size + 1 + targetSize);
        unsigned all_history_size = isGlobalHist ? history_size:btb_size*history_size;
        unsigned all_table_size = isGlobalTable ? (2*std::pow(2,history_size)):(btb_size*2*std::pow(2,history_size));
        stats.size = all_tags_and_valid_bit + all_history_size + all_table_size;
    }
    void free_dynamic_memory(){
        if(!isGlobalHist){
            for (unsigned i = 0; i < btb_size; i++)
            {
                delete btb_array[i].history;
            }
        }else{
            delete btb_array[0].history;
        }
        if(!isGlobalTable){
            for (unsigned i = 0; i < btb_size; i++)
            {
                delete btb_array[i].table_arr;
            }
        }else{
            delete btb_array[0].table_arr;
        }
    }
    void get_stats(SIM_stats* curStats){
        calculate_predictor_hypothesis_size();
        *curStats = stats;
        free_dynamic_memory();
    }
};



btb* btb_instance;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
	btb_instance = new btb(btbSize,historySize,tagSize,fsmState,isGlobalHist,isGlobalTable,Shared);
	return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
	return btb_instance->predit(pc,dst);
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
	btb_instance->update_btb_entry(pc,targetPc,taken,pred_dst);
}

void BP_GetStats(SIM_stats *curStats){
	btb_instance->get_stats(curStats);
}

