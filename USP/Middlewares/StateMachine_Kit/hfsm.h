/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    hfsm.h
 * @author  lrc
 * @brief   Header file
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#pragma once

#ifdef __cplusplus

namespace hfsm
{

  /**
   * @brief 事件类
   *
   */
  class Event_t
  {
  public:
    Event_t() {}
    ~Event_t() {}
    Event_t(const Event_t &event) { *this = event; }
    bool operator==(const Event_t &event) { return this == &event; }
    bool operator!=(const Event_t &event) { return this != &event; }
  };

  class State_t;

  /**
   * @brief 状态转换关系
   *
   */
  class Trans_t
  {
  public:
    Trans_t() {}
    ~Trans_t() {}
    Trans_t(const Trans_t &trans) { *this = trans; }
    // 建立从_fState到_tState的关系，转换条件为_Event_t发生
    Trans_t(State_t *_fState, Event_t *_event, State_t *_tState)
    {
      pfState = _fState, pEvent = _event, ptState = _tState;
    }

    /* 链表节点 */
    Trans_t *next = nullptr;

    // 判断边（转换关系）的存在性， 1表示存在，0表示不存在
    bool checkTrans_t(State_t *_fState, Event_t *_event)
    {
      if (pfState == nullptr)
        return (pEvent == _event); // 若_fState不存在且事件存在，说明任意状态下该事件均能触发目标状态
      else
        return ((pfState == _fState) && (pEvent == _event));
    }

    State_t *getAimState(State_t *_fState, Event_t *_event)
    {
      if (checkTrans_t(_fState, _event))
        return ptState;
      else
        return nullptr;
    }

  private:
    State_t *pfState = nullptr;
    State_t *ptState = nullptr;
    Event_t *pEvent = nullptr;
  };

  /**
   * @brief 转换链表
   *
   */
  class TransList_t
  {
  public:
    TransList_t() { head = nullptr; }
    ~TransList_t() { /*clear();*/ }
    // 清除整个列表
    void clear()
    {
      Trans_t *p = head;
      while (head != nullptr)
      {
        p = head;
        head = head->next;
        delete (p);
      }
    }
    // 表头添加新元素
    void add(Trans_t *t)
    {
      if (head == nullptr)
        head = t;
      else
      {
        t->next = head->next;
        head->next = t;
      }
    }
    // 下标i从0开始
    Trans_t *get(short i)
    {
      Trans_t *p = head;
      while (p != nullptr && i-- > 0)
        p = p->next;
      return p;
    }
    short len()
    {
      Trans_t *p = head;
      short len = 0;
      while (p != nullptr)
        len++, p = p->next;
      return len;
    }

  private:
    Trans_t *head = nullptr;
  };

  /**
   * @brief 状态类
   *
   */
  class State_t
  {
  public:
    State_t() {}
    State_t(const State_t &state) { *this = state; }
    ~State_t()
    {
      transList.~TransList_t();
    }

    // 添加本状态的子状态
    void addSon(State_t *_state)
    {
      if (pHead == nullptr)
        pHead = _state;
      else
      {
        _state->pNext = pHead->pNext;
        pHead->pNext = _state;
      }
    }

    // 添加子状态之间的转换关系， 注意，该接口内部使用了new， 需要动态开辟内存支持
    void addSonTrans(State_t *_pState, Event_t *_event, State_t *_tState)
    {
      Trans_t *trans = new Trans_t(_pState, _event, _tState);
      transList.add(trans);
    }

    // 添加子状态之间的转换关系
    void addSonTrans(Trans_t *trans)
    {
      transList.add(trans);
    }

    // 设置初始子状态
    void setInitialState(State_t *_pSonState)
    {
      pSonState = _pSonState;
    }

    // 本状态的进入动作（短期动作）, 应当立马进入持续动作
    void enterAction()
    {
      if (callbackEnterAction != nullptr)
        callbackEnterAction();
      action();
    }
    // 本状态的动作（长期动作）
    void action()
    {
      if (callbackAction != nullptr)
        callbackAction(&(this->pSonEvent)); // 在本状态的动作中改变子层次的事件（具体怎么改只能从用户回调函数中决定）
      if (pSonState != nullptr)
      {
        if (pSonEvent != ppSonEvent && pSonEvent != nullptr) // 子层次事件发生改变
        {
          transmit();
          ppSonEvent = pSonEvent;
        }
        else
          pSonState->action();
      }
    }

    // 本状态的退出动作（短期动作）
    void exitAction()
    {
      if (callbackExitAction != nullptr)
        callbackExitAction();
    }

    bool operator==(const State_t &state) { return this == &state; }
    bool operator!=(const State_t &state) { return this != &state; }

    /*注册动作回调函数*/
    void regisEnterAction(void (*callback)()) { callbackEnterAction = callback; }
    void regisAction(void (*callback)(Event_t **pEvent)) { callbackAction = callback; }
    void regisExitAction(void (*callback)()) { callbackExitAction = callback; }

    State_t *pSonState = nullptr; // 如果有子状态，指向当前子状态
    Event_t *pSonEvent = nullptr; // 如果有子状态，记录子层次的当前事件，其可在本状态的动作函数中改变

  private:
    TransList_t transList; // 状态转换列表

    Event_t *ppSonEvent = nullptr; // 子层次的上一个事件，用于判断是否进行新一轮子层次的状态转换

    State_t *pHead = nullptr; // 建立子状态链表
    State_t *pNext = nullptr; // 建立子状态链表

    void (*callbackEnterAction)() = nullptr;
    void (*callbackAction)(Event_t **pEvent) = nullptr; // 注意这是指针的指针
    void (*callbackExitAction)() = nullptr;

    // 状态转换函数，从当前状态变为目标状态
    void transmit()
    {
      State_t *pNextSonState = nullptr;
      for (short i = 0; i < transList.len() && pNextSonState == nullptr; i++)
        pNextSonState = transList.get(i)->getAimState(pSonState, pSonEvent);

      if (pNextSonState != nullptr)
      {
        pSonState->exitAction();
        pSonState = pNextSonState;
        pSonState->enterAction();
      }
    }
  };

};

/* Exported function declarations --------------------------------------------*/

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
